#include "camera_source_control.h"
#include <xgc2/xrpc/http.hpp>
#include <cassert>
#include <filesystem>
#include <future>
#include <iostream>
#include <unistd.h>
using namespace gazebo_sim_camera;
using namespace xgc2::xrpc;
int main() {
  char directory[] = "/tmp/sol6-camera-XXXXXX";
  assert(::mkdtemp(directory));
  const auto endpoint = std::string(directory) + "/source.sock";
  const auto instance = new_instance_id();
  {
    auto host = CameraSourceControlHost::Acquire(endpoint, "fixture-target", instance);
    auto second = CameraSourceControlHost::Acquire(endpoint, "fixture-target");
    assert(host == second);
    bool rejected = false;
    try { CameraSourceControlHost::Acquire(endpoint + "-other", "fixture-target"); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { CameraSourceControlHost::Acquire(endpoint, "other-target"); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { CameraSourceControlHost::Acquire(endpoint, "fixture-target", new_instance_id()); } catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
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
    assert(value.size() == 5);
    assert(value["service"] == "camera-source" && value["api_version"] == "v1");
    assert(value["instance_id"].asString() == instance);
    assert(instance.size() == 32 && instance.find_first_not_of("0123456789abcdef") == std::string::npos);
    assert(value["ready"].isBool() && value["ready"].asBool());
    const auto facts = value["facts"];
    assert(facts.size() == 4 && facts["target_id"] == "fixture-target");
    assert(facts["sources"].isArray() && facts["sources"].empty());
    assert(facts["service_ref"] == host->ServiceRef());
    assert(facts["service_ref"]["instance_id"] == value["instance_id"]);
    assert(facts["service_ref"]["service"] == value["service"]);
    assert(facts["service_ref"]["api_version"] == value["api_version"]);
    assert(facts["service_ref"]["target_id"] == facts["target_id"]);
    assert(facts["service_ref"]["profile"] == "http.v1");
    assert(facts["service_ref"]["endpoint"]["kind"] == "unix");
    assert(facts["service_ref"]["endpoint"]["address"] == endpoint);
    assert(facts["limits"].size() == 6);
    assert(facts["limits"]["sources"] == 16 && facts["limits"]["concurrent_captures"] == 1);
    assert(facts["limits"]["connections"] == 8 && facts["limits"]["inflight"] == 4);
    assert(facts["limits"]["request_bytes"] == 65536 && facts["limits"]["response_bytes"] == 67108864);

    // A readiness wait is unbound and already satisfied while the host serves,
    // including before the first source registers. It must not sleep or block IO.
    auto waiting = std::async(std::launch::async, [&] {
      return call(discovery, "/v1/describe?wait_ready_ms=30000");
    });
    assert(waiting.wait_for(std::chrono::seconds(1)) == std::future_status::ready);
    result = waiting.get();
    assert(result.status == 200 && ParseControlJSON(result.body) == value);
    assert(call(discovery, "/v1/describe?wait_ready_ms=0").status == 200);
    assert(call(discovery, "/v1/describe?wait_ready_ms=1").status == 200);
    for (const auto query : {"", "other=1", "wait_ready_ms=", "wait_ready_ms=-1",
        "wait_ready_ms=30001", "wait_ready_ms=999999999999999999999999", "wait_ready_ms=abc",
        "wait_ready_ms=1.5", "wait_ready_ms=+1", "wait_ready_ms=01", "wait_ready_ms=%31",
        "wait_ready_ms=1&other=2", "wait_ready_ms=1&wait_ready_ms=2", "wait_ready_ms=1?x=2"}) {
      result = call(discovery, std::string("/v1/describe?") + query);
      assert(result.status == 400);
      assert(ParseControlJSON(result.body)["error"]["code"] == "invalid_argument");
    }
    for (const auto id : {"front", "world"}) host->Register(id, [id](auto operation, const auto &, const auto &, auto reply) {
      Json::Value response;
      response["source_id"] = id;
      response["operation"] = operation;
      reply.complete(JSONResponse(response));
    }, [] {});
    result = call(discovery, "/v1/describe?wait_ready_ms=30000");
    value = ParseControlJSON(result.body);
    assert(result.status == 200 && value["ready"].asBool());
    assert(value["facts"]["sources"].size() == 2);
    assert(value["facts"]["sources"][0] == "front" && value["facts"]["sources"][1] == "world");
    assert(call(discovery, "/v1/media/sources/world").status == 409);
    assert(call(discovery, "/v1/media/sources").status == 409);
    HttpClient client(endpoint, {}, instance);
    result = call(client, "/v1/media/sources");
    const auto roster = ParseControlJSON(result.body);
    assert(result.status == 200 && roster.size() == 2);
    assert(roster["sources"] == value["facts"]["sources"] && roster["limits"] == facts["limits"]);
    result = call(client, "/v1/runtime-policy");
    assert(result.status == 404 && ParseControlJSON(result.body)["error"]["code"] == "not_found");
    assert(call(client, "/v1/media/sources/world").status == 404);
    result = call(client, "/v1/media/sources/world/status");
    assert(result.status == 200 && ParseControlJSON(result.body)["source_id"] == "world");
    assert(call(client, "/v1/set-active", "{}").status == 404);
    assert(call(client, "/v1/media/sources/world/start", "{\"persist\":false,\"persist\":true}").status == 400);
    assert(call(client, "/v1/media/sources/world/start", "{} {}").status == 400);
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
    result = call(discovery, "/v1/describe?wait_ready_ms=30000");
    value = ParseControlJSON(result.body);
    assert(result.status == 200 && value["ready"].asBool() && value["facts"]["sources"].empty());
    discovery.close();
    client.close();
  }
  assert(!std::filesystem::exists(endpoint));
  std::filesystem::remove_all(directory);
  std::cout << "camera shared XRPC host: envelope/unbound readiness/invalid queries/removed route/"
      "multi-source/fencing/strict JSON/capture bound/lifetime PASS\n";
}
