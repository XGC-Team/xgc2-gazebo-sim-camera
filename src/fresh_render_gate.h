#pragma once

#include <cstdint>
#include <optional>

namespace gazebo_sim_camera {

enum class FreshRenderDecision {
  kAccept,
  kDiscardAndReset,
  kWaitForSourceTimeChange,
};

// A reactivated Gazebo CameraSensor can invoke its render-target listener once
// with the texture and LastMeasurementTime from before it was deactivated.
// This render-thread-only gate makes an activation generation establish a
// source-time sample, then admits only a callback whose time has changed. The
// comparison is deliberately not ordered because simulation time can reset.
class FreshRenderGate {
 public:
  FreshRenderDecision Observe(
      std::uint64_t requestedGeneration,
      std::int64_t sourceTimeNanoseconds) {
    if (requestedGeneration != observedGeneration_) {
      observedGeneration_ = requestedGeneration;
      sourceTimeFloorNanoseconds_ = sourceTimeNanoseconds;
      return FreshRenderDecision::kDiscardAndReset;
    }
    if (!sourceTimeFloorNanoseconds_) {
      return FreshRenderDecision::kAccept;
    }
    if (sourceTimeNanoseconds == *sourceTimeFloorNanoseconds_) {
      return FreshRenderDecision::kWaitForSourceTimeChange;
    }
    sourceTimeFloorNanoseconds_.reset();
    return FreshRenderDecision::kAccept;
  }

 private:
  std::uint64_t observedGeneration_ = 0;
  std::optional<std::int64_t> sourceTimeFloorNanoseconds_;
};

}  // namespace gazebo_sim_camera
