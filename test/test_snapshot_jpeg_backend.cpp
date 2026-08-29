#include "snapshot_jpeg_backend.h"

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

namespace gazebo_sim_camera {
namespace {

TEST(SnapshotJpegBackend, ParsesOnlyStablePolicies) {
  EXPECT_EQ(ParseSnapshotJpegPolicy("auto"), SnapshotJpegPolicy::kAuto);
  EXPECT_EQ(ParseSnapshotJpegPolicy("hardware"), SnapshotJpegPolicy::kHardware);
  EXPECT_EQ(ParseSnapshotJpegPolicy("cpu"), SnapshotJpegPolicy::kCPU);
  EXPECT_FALSE(ParseSnapshotJpegPolicy("nvenc"));
  EXPECT_FALSE(ParseSnapshotJpegPolicy(""));
}

TEST(SnapshotJpegBackend, AutoPrefersHardwareAndFallsBackToCPU) {
  const auto accelerated = SelectSnapshotJpegBackend(
      SnapshotJpegPolicy::kAuto, true, "nvjpeg-cuda");
  EXPECT_TRUE(accelerated.available);
  EXPECT_TRUE(accelerated.useHardware);
  EXPECT_TRUE(accelerated.allowCPUFallback);
  EXPECT_EQ(accelerated.backend, "nvjpeg-cuda");

  const auto portable = SelectSnapshotJpegBackend(
      SnapshotJpegPolicy::kAuto, false);
  EXPECT_TRUE(portable.available);
  EXPECT_FALSE(portable.useHardware);
  EXPECT_TRUE(portable.allowCPUFallback);
  EXPECT_TRUE(
      portable.backend == "libjpeg-turbo" || portable.backend == "libjpeg");
  EXPECT_EQ(portable.hardwareState, "unavailable");
}

TEST(SnapshotJpegBackend, HardwareIsStrictAndCPUDoesNotProbeIt) {
  const auto missing = SelectSnapshotJpegBackend(
      SnapshotJpegPolicy::kHardware, false);
  EXPECT_FALSE(missing.available);
  EXPECT_FALSE(missing.allowCPUFallback);
  EXPECT_NE(missing.error.find("requires a hardware backend"), std::string::npos);

  const auto cpu = SelectSnapshotJpegBackend(
      SnapshotJpegPolicy::kCPU, true, "unexpected-hardware");
  EXPECT_TRUE(cpu.available);
  EXPECT_FALSE(cpu.useHardware);
  EXPECT_FALSE(cpu.allowCPUFallback);
  EXPECT_EQ(cpu.hardwareState, "not-requested");
}

TEST(SnapshotJpegBackend, LoadsAndExecutesTheVersionedHardwareABI) {
  SnapshotJpegHardwareBackend backend;
  ASSERT_TRUE(backend.Open(XGC_TEST_JPEG_HARDWARE_LIBRARY)) << backend.error();
  EXPECT_TRUE(backend.available());
  EXPECT_EQ(backend.backend(), "fake-hardware-jpeg");
  const std::array<std::uint8_t, 3> rgb{1, 2, 3};
  const auto encoded = backend.Encode(rgb.data(), rgb.size(), 1, 1, 90);
  EXPECT_TRUE(encoded.error.empty()) << encoded.error;
  EXPECT_EQ(encoded.backend, "fake-hardware-jpeg");
  EXPECT_EQ(encoded.bytes.front(), 0xff);
  EXPECT_EQ(encoded.bytes.back(), 0xd9);
}

TEST(SnapshotJpegBackend, CPUProducesAStandardJPEG) {
  constexpr unsigned int width = 8;
  constexpr unsigned int height = 4;
  std::vector<std::uint8_t> rgb(width * height * 3);
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const std::size_t offset = (static_cast<std::size_t>(y) * width + x) * 3;
      rgb[offset] = static_cast<std::uint8_t>(x * 20);
      rgb[offset + 1] = static_cast<std::uint8_t>(y * 50);
      rgb[offset + 2] = static_cast<std::uint8_t>(255 - x * 20);
    }
  }

  const auto encoded = EncodeSnapshotJpegCPU(
      rgb.data(), rgb.size(), width, height, 90);
  ASSERT_TRUE(encoded.error.empty()) << encoded.error;
  ASSERT_GE(encoded.bytes.size(), 4u);
  EXPECT_EQ(encoded.bytes[0], 0xff);
  EXPECT_EQ(encoded.bytes[1], 0xd8);
  EXPECT_EQ(encoded.bytes[encoded.bytes.size() - 2], 0xff);
  EXPECT_EQ(encoded.bytes.back(), 0xd9);
}

TEST(SnapshotJpegBackend, CPURejectsMalformedRGBWithoutReadingIt) {
  const std::array<std::uint8_t, 3> rgb{0, 0, 0};
  const auto encoded = EncodeSnapshotJpegCPU(
      rgb.data(), rgb.size(), 8, 4, 90);
  EXPECT_TRUE(encoded.bytes.empty());
  EXPECT_FALSE(encoded.error.empty());
}

TEST(SnapshotJpegBackend, PreservesGazeboRenderTextureRowOrder) {
  const std::vector<std::uint8_t> renderTextureRows{
      0, 0, 255, 255, 255, 255,
      255, 0, 0, 0, 255, 0,
  };
  std::vector<std::uint8_t> imageRows;
  ASSERT_TRUE(CopyGazeboPBOToImageOrder(
      renderTextureRows.data(), renderTextureRows.size(), 2, 2, &imageRows));
  EXPECT_EQ(imageRows, (std::vector<std::uint8_t>{
      0, 0, 255, 255, 255, 255,
      255, 0, 0, 0, 255, 0,
  }));
}

}  // namespace
}  // namespace gazebo_sim_camera

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
