#pragma once

#include <chrono>
#include <vector>
#include <json/json.h>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace xgc2 { namespace xrpc { class HttpServer; class RuntimePolicy; } }

namespace gazebo_sim_camera {

using ControlClock = std::chrono::steady_clock;
struct ControlRequest {
  std::string method, target, body, request_id;
  ControlClock::time_point deadline;
};
struct ControlResponse {
  int status = 200;
  std::vector<std::pair<std::string, std::string>> headers;
  std::string body;
};
class ControlReply {
 public:
  struct State;
  ControlReply() = default;
  explicit ControlReply(std::shared_ptr<State> state) : state_(std::move(state)) {}
  bool complete(ControlResponse response) const;
  bool cancelled() const noexcept;
 private:
  std::shared_ptr<State> state_;
};
ControlResponse ControlError(int status, const std::string &code, const std::string &message);
std::string NewControlIdentifier();

Json::Value ParseControlJSON(const std::string &body);
std::string ControlJSON(const Json::Value &value);
ControlResponse JSONResponse(const Json::Value &value, int status = 200);
bool SafeSourceIdentifier(const std::string &value);

// One fixed HTTP executor for all camera sensors in a Gazebo process. Sensor
// removal unregisters the callbacks before native plugin storage is released.
// No listener or executor is added when a second sensor joins the world.
class CameraSourceControlHost {
 public:
  static constexpr std::size_t kSources = 16;
  using Handler = std::function<void(const std::string &, const ControlRequest &,
                                    const Json::Value &, ControlReply)>;
  using Flush = std::function<void()>;
  static std::shared_ptr<CameraSourceControlHost> Acquire(
      const std::string &path, const std::string &target_id, const std::string &instance_id = "");
  ~CameraSourceControlHost();
  void Register(const std::string &source_id, Handler handler, Flush flush);
  void Unregister(const std::string &source_id);
  void Wake() noexcept;
  bool ReserveCapture() noexcept;
  void ReleaseCapture() noexcept;
  Json::Value ServiceRef() const;

 private:
  CameraSourceControlHost(const std::string &path, const std::string &target_id, const std::string &instance_id);
  void Dispatch(ControlRequest request, ControlReply reply);
  void FlushSources();
  struct Source { std::string id; Handler handler; Flush flush; };
  std::array<Source, kSources> sources_{};
  std::mutex sources_mutex_;
  std::string path_, instance_id_, target_id_;
  std::unique_ptr<xgc2::xrpc::HttpServer> server_;
  std::shared_ptr<const xgc2::xrpc::RuntimePolicy> policy_;
  std::thread executor_;
  std::atomic<bool> stopping_{false}, capture_reserved_{false};
};
}  // namespace gazebo_sim_camera
