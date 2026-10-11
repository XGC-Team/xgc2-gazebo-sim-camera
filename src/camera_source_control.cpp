#include "camera_source_control.h"

#include <algorithm>
#include <stdexcept>

namespace gazebo_sim_camera {
namespace {
std::mutex host_mutex;
std::weak_ptr<CameraSourceControlHost> shared_host;
}

struct ControlReply::State { xgc2::xrpc::HttpReply reply; };
bool ControlReply::complete(ControlResponse response) const {
  if (!state_) return false;
  xgc2::xrpc::HttpResponse native;
  native.status = response.status;
  native.headers = std::move(response.headers);
  native.body = std::move(response.body);
  return state_->reply.complete(std::move(native));
}
bool ControlReply::cancelled() const noexcept { return !state_ || state_->reply.cancelled(); }
ControlResponse ControlError(int status, const std::string &code, const std::string &message) {
  auto response = xgc2::xrpc::http_error(status, code, message);
  return {response.status, std::move(response.headers), std::move(response.body)};
}
std::string NewControlIdentifier() { return xgc2::xrpc::new_instance_id(); }

bool SafeSourceIdentifier(const std::string &value) {
  return !value.empty() && value.size() <= 128 &&
      std::all_of(value.begin(), value.end(), [](unsigned char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.';
      });
}

Json::Value ParseControlJSON(const std::string &body) {
  if (body.empty()) return Json::Value(Json::objectValue);
  Json::CharReaderBuilder settings;
  settings["allowComments"] = false;
  settings["allowTrailingCommas"] = false;
  settings["strictRoot"] = true;
  settings["failIfExtra"] = true;
  settings["rejectDupKeys"] = true;
  settings["stackLimit"] = 16;
  Json::Value result;
  std::string error;
  const auto reader = std::unique_ptr<Json::CharReader>(settings.newCharReader());
  if (!reader->parse(body.data(), body.data() + body.size(), &result, &error) ||
      !result.isObject()) throw std::invalid_argument("camera request must be a strict JSON object");
  return result;
}

std::string ControlJSON(const Json::Value &value) {
  Json::StreamWriterBuilder settings;
  settings["indentation"] = "";
  return Json::writeString(settings, value);
}

ControlResponse JSONResponse(const Json::Value &value, int status) {
  ControlResponse response;
  response.status = status;
  response.headers.emplace_back("Content-Type", "application/json");
  response.body = ControlJSON(value);
  return response;
}

std::shared_ptr<CameraSourceControlHost> CameraSourceControlHost::Acquire(
    const std::string &path, const std::string &target_id, const std::string &instance_id) {
  if (!SafeSourceIdentifier(target_id)) throw std::invalid_argument("explicit bounded control target identity is required");
  std::lock_guard<std::mutex> lock(host_mutex);
  if (auto host = shared_host.lock()) {
    if (host->path_ != path || host->target_id_ != target_id || (!instance_id.empty() && host->instance_id_ != instance_id))
      throw std::invalid_argument("all camera sensors in one world must share the same control binding");
    return host;
  }
  auto host = std::shared_ptr<CameraSourceControlHost>(new CameraSourceControlHost(path, target_id, instance_id));
  shared_host = host;
  return host;
}

CameraSourceControlHost::CameraSourceControlHost(const std::string &path,
    const std::string &target_id, const std::string &instance_id) : path_(path), target_id_(target_id),
    instance_id_(instance_id.empty() ? xgc2::xrpc::new_instance_id() : instance_id) {
  limits_.connections = 8;
  limits_.inflight = 4;
  limits_.header_bytes = 8192;
  limits_.request_bytes = 65536;
  limits_.response_bytes = 64U << 20U;
  limits_.request_timeout = std::chrono::seconds(10);
  limits_.header_timeout = std::chrono::seconds(5);
  limits_.idle_timeout = std::chrono::seconds(5);
  limits_.shutdown_timeout = std::chrono::seconds(1);
  xgc2::xrpc::UnixOptions endpoint;
  endpoint.path = path;
  server_ = std::make_unique<xgc2::xrpc::HttpServer>(endpoint,
      [this](auto request, auto reply) {
        ControlRequest domain{request.method, request.target, std::move(request.body), request.request_id, request.deadline};
        auto state = std::make_shared<ControlReply::State>();
        state->reply = reply;
        Dispatch(std::move(domain), ControlReply(std::move(state)));
      }, limits_,
      xgc2::xrpc::HttpIdentity{instance_id_, {"/v1/describe"}});
  server_->set_wakeup_handler([this] { FlushSources(); });
  executor_ = std::thread([this] {
    while (!stopping_.load()) {
      server_->poll(std::chrono::milliseconds(10));
      FlushSources(); // Domain deadlines, never external receipt polling.
    }
    server_->request_stop();
  });
}

CameraSourceControlHost::~CameraSourceControlHost() {
  stopping_.store(true);
  server_->wake();
  if (executor_.joinable()) executor_.join();
}

void CameraSourceControlHost::Register(const std::string &source_id,
    Handler handler, Flush flush) {
  if (!SafeSourceIdentifier(source_id)) throw std::invalid_argument("invalid camera source_id");
  std::lock_guard<std::mutex> lock(sources_mutex_);
  for (const auto &source : sources_)
    if (source.id == source_id) throw std::invalid_argument("duplicate camera source_id");
  for (auto &source : sources_) if (source.id.empty()) {
    source = Source{source_id, std::move(handler), std::move(flush)};
    return;
  }
  throw std::runtime_error("camera source capacity is exhausted (16)");
}

void CameraSourceControlHost::Unregister(const std::string &source_id) {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  for (auto &source : sources_) if (source.id == source_id) source = Source{};
}

void CameraSourceControlHost::Wake() noexcept { server_->wake(); }
bool CameraSourceControlHost::ReserveCapture() noexcept {
  bool expected = false;
  return capture_reserved_.compare_exchange_strong(expected, true);
}
void CameraSourceControlHost::ReleaseCapture() noexcept { capture_reserved_.store(false); }

Json::Value CameraSourceControlHost::ServiceRef() const {
  Json::Value ref;
  ref["target_id"] = target_id_;
  ref["service"] = "camera-source";
  ref["api_version"] = "v1";
  ref["instance_id"] = instance_id_;
  ref["profile"] = "http.v1";
  ref["endpoint"]["kind"] = "unix";
  ref["endpoint"]["address"] = path_;
  return ref;
}

void CameraSourceControlHost::FlushSources() {
  std::lock_guard<std::mutex> lock(sources_mutex_);
  for (const auto &source : sources_) if (source.flush) source.flush();
}

void CameraSourceControlHost::Dispatch(ControlRequest request,
    ControlReply reply) {
  try {
    const auto input = ParseControlJSON(request.body);
    std::lock_guard<std::mutex> lock(sources_mutex_);
    if (request.method == "GET" && (request.target == "/v1/describe" || request.target == "/v1/media/sources")) {
      if (!input.empty()) throw std::invalid_argument("discovery does not accept a body");
      Json::Value result;
      if (request.target == "/v1/describe") result["service_ref"] = ServiceRef();
      result["sources"] = Json::Value(Json::arrayValue);
      for (const auto &source : sources_) if (!source.id.empty()) result["sources"].append(source.id);
      result["limits"]["sources"] = Json::UInt(kSources);
      result["limits"]["concurrent_captures"] = 1;
      result["limits"]["connections"] = Json::UInt64(limits_.connections);
      result["limits"]["inflight"] = Json::UInt64(limits_.inflight);
      result["limits"]["request_bytes"] = Json::UInt64(limits_.request_bytes);
      result["limits"]["response_bytes"] = Json::UInt64(limits_.response_bytes);
      reply.complete(JSONResponse(result));
      return;
    }
    static const std::string prefix = "/v1/media/sources/";
    if (request.target.compare(0, prefix.size(), prefix) != 0) {
      reply.complete(ControlError(404, "not_found", "camera route not found"));
      return;
    }
    const auto separator = request.target.find('/', prefix.size());
    const auto id = request.target.substr(prefix.size(), separator == std::string::npos ?
        std::string::npos : separator - prefix.size());
    const auto operation = separator == std::string::npos ? "" : request.target.substr(separator + 1);
    if (operation.empty()) {
      reply.complete(ControlError(404, "not_found", "an explicit source domain route is required")); return;
    }
    if (!SafeSourceIdentifier(id)) throw std::invalid_argument("invalid camera source_id");
    for (const auto &source : sources_) if (source.id == id) {
      source.handler(operation, request, input, reply);
      return;
    }
    reply.complete(ControlError(404, "not_found", "camera source not found"));
  } catch (const std::invalid_argument &error) {
    reply.complete(ControlError(400, "invalid_argument", error.what()));
  } catch (const std::exception &error) {
    reply.complete(ControlError(500, "internal", error.what()));
  }
}
}  // namespace gazebo_sim_camera
