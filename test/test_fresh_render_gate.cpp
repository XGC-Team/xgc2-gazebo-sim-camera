#include "fresh_render_gate.h"

#include <gtest/gtest.h>

namespace {

using gazebo_sim_camera::FreshRenderDecision;
using gazebo_sim_camera::FreshRenderGate;

TEST(FreshRenderGate, RejectsDormantTextureUntilSourceTimeAdvances) {
  FreshRenderGate gate;

  EXPECT_EQ(gate.Observe(0, 102'627'000'000LL),
            FreshRenderDecision::kAccept);
  EXPECT_EQ(gate.Observe(1, 102'627'000'000LL),
            FreshRenderDecision::kDiscardAndReset);
  EXPECT_EQ(gate.Observe(1, 102'627'000'000LL),
            FreshRenderDecision::kWaitForSourceTimeChange);
  EXPECT_EQ(gate.Observe(1, 137'512'000'000LL),
            FreshRenderDecision::kAccept);
  EXPECT_EQ(gate.Observe(1, 137'545'000'000LL),
            FreshRenderDecision::kAccept);
}

TEST(FreshRenderGate, EveryActivationCreatesANewSourceTimeFloor) {
  FreshRenderGate gate;

  EXPECT_EQ(gate.Observe(1, 20'000'000'000LL),
            FreshRenderDecision::kDiscardAndReset);
  EXPECT_EQ(gate.Observe(1, 20'050'000'000LL),
            FreshRenderDecision::kAccept);
  EXPECT_EQ(gate.Observe(2, 55'000'000'000LL),
            FreshRenderDecision::kDiscardAndReset);
  EXPECT_EQ(gate.Observe(2, 54'000'000'000LL),
            FreshRenderDecision::kAccept);
  EXPECT_EQ(gate.Observe(2, 54'050'000'000LL),
            FreshRenderDecision::kAccept);
}

}  // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
