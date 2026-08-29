#include "snapshot_jpeg_backend.h"

#include <jpeglib.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

std::vector<std::uint8_t> DecodeJPEG(
    const std::vector<std::uint8_t> &jpeg,
    unsigned int expectedWidth,
    unsigned int expectedHeight) {
  jpeg_decompress_struct decoder{};
  jpeg_error_mgr errors{};
  decoder.err = jpeg_std_error(&errors);
  jpeg_create_decompress(&decoder);
  jpeg_mem_src(
      &decoder, const_cast<unsigned char *>(jpeg.data()), jpeg.size());
  jpeg_read_header(&decoder, TRUE);
  decoder.out_color_space = JCS_RGB;
  jpeg_start_decompress(&decoder);
  if (decoder.output_width != expectedWidth ||
      decoder.output_height != expectedHeight ||
      decoder.output_components != 3) {
    jpeg_destroy_decompress(&decoder);
    return {};
  }
  std::vector<std::uint8_t> rgb(
      static_cast<std::size_t>(expectedWidth) * expectedHeight * 3);
  const std::size_t rowBytes = static_cast<std::size_t>(expectedWidth) * 3;
  while (decoder.output_scanline < decoder.output_height) {
    JSAMPROW row = rgb.data() +
        static_cast<std::size_t>(decoder.output_scanline) * rowBytes;
    jpeg_read_scanlines(&decoder, &row, 1);
  }
  jpeg_finish_decompress(&decoder);
  jpeg_destroy_decompress(&decoder);
  return rgb;
}

double MeanAbsoluteError(
    const std::vector<std::uint8_t> &left,
    const std::vector<std::uint8_t> &right,
    unsigned int width,
    unsigned int height,
    bool flipVertical,
    bool flipHorizontal,
    bool swapRedBlue) {
  const std::size_t rowBytes = static_cast<std::size_t>(width) * 3;
  double total = 0.0;
  for (unsigned int row = 0; row < height; ++row) {
    const unsigned int rightRow = flipVertical ? height - row - 1 : row;
    for (unsigned int column = 0; column < width; ++column) {
      const unsigned int rightColumn = flipHorizontal ? width - column - 1 : column;
      for (unsigned int channel = 0; channel < 3; ++channel) {
        const unsigned int rightChannel = swapRedBlue ? 2 - channel : channel;
        total += std::abs(
            static_cast<int>(left[
                (static_cast<std::size_t>(row) * width + column) * 3 + channel]) -
            static_cast<int>(right[
                (static_cast<std::size_t>(rightRow) * width + rightColumn) * 3 +
                rightChannel]));
      }
    }
  }
  return total / static_cast<double>(left.size());
}

double Percentile(std::vector<double> values, double fraction) {
  std::sort(values.begin(), values.end());
  const std::size_t index = std::min(
      values.size() - 1,
      static_cast<std::size_t>(fraction * static_cast<double>(values.size())));
  return values[index];
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: snapshot_jpeg_hardware_smoke /absolute/path/to/backend.so\n";
    return 2;
  }
  constexpr unsigned int width = 3840;
  constexpr unsigned int height = 2160;
  std::vector<std::uint8_t> rgb(
      static_cast<std::size_t>(width) * height * 3);
  for (unsigned int row = 0; row < height; ++row) {
    for (unsigned int column = 0; column < width; ++column) {
      const std::size_t offset =
          (static_cast<std::size_t>(row) * width + column) * 3;
      rgb[offset] = static_cast<std::uint8_t>(column * 255u / (width - 1));
      rgb[offset + 1] = static_cast<std::uint8_t>(row * 255u / (height - 1));
      rgb[offset + 2] = static_cast<std::uint8_t>(
          (static_cast<std::uint64_t>(row) + column) * 255u /
          (width + height - 2));
    }
  }

  gazebo_sim_camera::SnapshotJpegHardwareBackend hardware;
  if (!hardware.Open(argv[1])) {
    std::cerr << hardware.error() << '\n';
    return 1;
  }
  // Warm reusable nvJPEG state and device storage before measuring.
  auto warm = hardware.Encode(rgb.data(), rgb.size(), width, height, 90);
  if (!warm.error.empty()) {
    std::cerr << warm.error << '\n';
    return 1;
  }
  auto cpuWarm = gazebo_sim_camera::EncodeSnapshotJpegCPU(
      rgb.data(), rgb.size(), width, height, 90);
  if (!cpuWarm.error.empty()) {
    std::cerr << cpuWarm.error << '\n';
    return 1;
  }

  std::vector<double> hardwareTimings;
  std::vector<double> cpuTimings;
  gazebo_sim_camera::SnapshotJpegEncodeResult accelerated;
  gazebo_sim_camera::SnapshotJpegEncodeResult cpu;
  for (int iteration = 0; iteration < 20; ++iteration) {
    const auto hardwareStarted = std::chrono::steady_clock::now();
    accelerated = hardware.Encode(rgb.data(), rgb.size(), width, height, 90);
    hardwareTimings.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - hardwareStarted).count());
    if (!accelerated.error.empty()) {
      std::cerr << accelerated.error << '\n';
      return 1;
    }
    const auto cpuStarted = std::chrono::steady_clock::now();
    cpu = gazebo_sim_camera::EncodeSnapshotJpegCPU(
        rgb.data(), rgb.size(), width, height, 90);
    cpuTimings.push_back(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - cpuStarted).count());
    if (!cpu.error.empty()) {
      std::cerr << cpu.error << '\n';
      return 1;
    }
  }
  const auto decoded = DecodeJPEG(accelerated.bytes, width, height);
  if (decoded.size() != rgb.size()) {
    std::cerr << "nvJPEG result did not decode to source dimensions\n";
    return 1;
  }
  const double imageError = MeanAbsoluteError(
      decoded, rgb, width, height, false, false, false);
  const double verticalError = MeanAbsoluteError(
      decoded, rgb, width, height, true, false, false);
  const double horizontalError = MeanAbsoluteError(
      decoded, rgb, width, height, false, true, false);
  const double swappedError = MeanAbsoluteError(
      decoded, rgb, width, height, false, false, true);
  if (imageError > 8.0 || imageError >= verticalError * 0.5 ||
      imageError >= horizontalError * 0.5 || imageError >= swappedError * 0.5) {
    std::cerr << "nvJPEG color/orientation mismatch: mae=" << imageError
              << " vertical_mae=" << verticalError
              << " horizontal_mae=" << horizontalError
              << " swapped_mae=" << swappedError << '\n';
    return 1;
  }
  const auto cpuDecoded = DecodeJPEG(cpu.bytes, width, height);
  if (cpuDecoded.size() != rgb.size()) {
    std::cerr << "CPU JPEG did not decode to source dimensions\n";
    return 1;
  }
  const double cpuImageError = MeanAbsoluteError(
      cpuDecoded, rgb, width, height, false, false, false);
  const double encoderDifference = MeanAbsoluteError(
      decoded, cpuDecoded, width, height, false, false, false);
  if (cpuImageError > 8.0 || encoderDifference > 4.0) {
    std::cerr << "CPU/GPU JPEG mismatch: cpu_mae=" << cpuImageError
              << " encoder_mae=" << encoderDifference << '\n';
    return 1;
  }

  std::cout << "hardware_backend=" << accelerated.backend
            << " hardware_p50_ms=" << Percentile(hardwareTimings, 0.50)
            << " hardware_p95_ms=" << Percentile(hardwareTimings, 0.95)
            << " hardware_bytes=" << accelerated.bytes.size()
            << " decoded_mae=" << imageError
            << " cpu_backend=" << cpu.backend
            << " cpu_p50_ms=" << Percentile(cpuTimings, 0.50)
            << " cpu_p95_ms=" << Percentile(cpuTimings, 0.95)
            << " cpu_bytes=" << cpu.bytes.size() << '\n';
  return 0;
}
