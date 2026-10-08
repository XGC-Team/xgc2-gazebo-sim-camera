#include "camera_source_control.h"
#include <xgc2/xrpc/http.hpp>
#include <cassert>
#include <filesystem>
#include <iostream>
#include <unistd.h>
using namespace gazebo_sim_camera;
using namespace xgc2::xrpc;
int main() {
  char directory[] = "/tmp/sol6-camera-XXXXXX";
  assert(::mkdtemp(directory));
  const auto endpoint = std::string(directory) + "/source.sock";
  {
    auto host = CameraSourceControlHost::Acquire(endpoint, "fixture-target", "camera-test-instance");
    auto second = CameraSourceControlHost::Acquire(endpoint, "fixture-target");
    assert(host == second);
    bool rejected = false;
    try { CameraSourceControlHost::Acquire(endpoint + "-other", "fixture-target"); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
    for (const auto id : {"front", "world"}) host->Register(id, [id, host](auto operation, const auto &, const auto &, auto reply) {
      Json::Value response;
      response["source_id"] = id;
      response["operation"] = operation;
      reply.complete(JSONResponse(response));
    }, [] {});
    HttpClient discovery(endpoint);
    const auto call = [](auto &client, const std::string &path, const std::string &body = "") {
      HttpRequest request;
      request.method = body.empty() ? "GET" : "POST";
      request.target = path;
      request.request_id = new_instance_id();
      request.body = body;
      return client.call(std::move(request), Clock::now() + std::chrono::seconds(2));
    };
    auto result = call(discovery, "/v1/describe");
    assert(result.status == 200);
    auto value = ParseControlJSON(result.body);
    assert(value["sources"].size() == 2);
    assert(value["service_ref"]["instance_id"].asString() == "camera-test-instance");
    assert(call(discovery, "/v1/media/sources/world").status == 409);
    HttpClient client(endpoint, {}, "camera-test-instance");
    assert(call(client, "/v1/media/sources/world").status == 404);
    result = call(client, "/v1/media/sources/world/status");
    assert(result.status == 200 && ParseControlJSON(result.body)["source_id"] == "world");
    assert(call(client, "/v1/set-active", "{}").status == 404);
    assert(call(client, "/v1/media/sources/world/start", "{\"persist\":false,\"persist\":true}").status == 400);
    assert(call(client, "/v1/media/sources/world/start", "{} garbage").status == 400);
    assert(call(client, "/v1/media/sources/absent").status == 404);
    assert(host->ReserveCapture());
    assert(!second->ReserveCapture());
    host->ReleaseCapture();
    assert(second->ReserveCapture());
    second->ReleaseCapture();
    host->Unregister("world");
    assert(call(client, "/v1/media/sources/world").status == 404);
    host->Unregister("front");
    for (int i = 0; i < 16; ++i) host->Register("camera-" + std::to_string(i), [](auto, const auto &, const auto &, auto reply) { reply.complete(JSONResponse(Json::Value(Json::objectValue))); }, [] {});
    rejected = false;
    try { host->Register("overflow", {}, {}); } catch (const std::runtime_error &) { rejected = true; }
    assert(rejected);
    for (int i = 0; i < 16; ++i) host->Unregister("camera-" + std::to_string(i));
    discovery.close();
    client.close();
  }
  assert(!std::filesystem::exists(endpoint));
  std::filesystem::remove_all(directory);
  std::cout << "camera shared XRPC host: multi-source/fencing/strict JSON/capture bound/lifetime PASS\n";
}
