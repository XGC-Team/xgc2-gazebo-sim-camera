#include "snapshot_jpeg_backend.h"
#include "snapshot_jpeg_hardware_abi.h"

#include <jpeglib.h>

#include <dlfcn.h>

#include <csetjmp>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <memory>

namespace gazebo_sim_camera {
namespace {

struct JPEGErrorManager {
  jpeg_error_mgr manager;
  jmp_buf jumpBuffer;
};

extern "C" void JPEGErrorExit(j_common_ptr info) {
  auto *error = reinterpret_cast<JPEGErrorManager *>(info->err);
  longjmp(error->jumpBuffer, 1);
}

const char *CPUBackendName() {
#ifdef LIBJPEG_TURBO_VERSION
  return "libjpeg-turbo";
#else
  return "libjpeg";
#endif
}

}  // namespace

SnapshotJpegHardwareBackend::~SnapshotJpegHardwareBackend() {
  Close();
}

bool SnapshotJpegHardwareBackend::Open(const std::string &library) {
  Close();
  if (library.empty()) {
    error_ = "snapshot JPEG hardware library is empty";
    return false;
  }
  library_ = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library_) {
    const char *message = dlerror();
    error_ = "hardware JPEG module is unavailable";
    if (message && *message) {
      error_ += ": ";
      error_ += message;
    }
    return false;
  }
  dlerror();
  auto create = reinterpret_cast<XGCSnapshotJpegHardwareCreateV1>(
      dlsym(library_, XGC_SNAPSHOT_JPEG_HARDWARE_CREATE_SYMBOL_V1));
  if (!create || dlerror() != nullptr) {
    error_ = "hardware JPEG module does not export the v1 factory";
    Close();
    return false;
  }
  auto encoder = std::make_unique<XGCSnapshotJpegHardwareEncoderV1>();
  char error[512] = {};
  if (create(
          XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1, encoder.get(), error,
          sizeof(error)) != 0 ||
      encoder->abi_version != XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1 ||
      encoder->struct_size < sizeof(XGCSnapshotJpegHardwareEncoderV1) ||
      !encoder->backend_name || !*encoder->backend_name ||
      std::char_traits<char>::length(encoder->backend_name) > 64 ||
      !encoder->encode_rgb || !encoder->release_jpeg || !encoder->destroy) {
    error_ = *error ? error : "hardware JPEG module failed ABI preflight";
    if (encoder->destroy && encoder->context) {
      encoder->destroy(encoder->context);
    }
    Close();
    return false;
  }
  backend_ = encoder->backend_name;
  encoderStorage_ = encoder.release();
  error_.clear();
  return true;
}

void SnapshotJpegHardwareBackend::Close() {
  auto *encoder = static_cast<XGCSnapshotJpegHardwareEncoderV1 *>(
      encoderStorage_);
  if (encoder) {
    if (encoder->destroy) {
      encoder->destroy(encoder->context);
    }
    delete encoder;
    encoderStorage_ = nullptr;
  }
  if (library_) {
    dlclose(library_);
    library_ = nullptr;
  }
  backend_.clear();
}

bool SnapshotJpegHardwareBackend::available() const {
  return library_ != nullptr && encoderStorage_ != nullptr;
}

const std::string &SnapshotJpegHardwareBackend::backend() const {
  return backend_;
}

const std::string &SnapshotJpegHardwareBackend::error() const {
  return error_;
}

SnapshotJpegEncodeResult SnapshotJpegHardwareBackend::Encode(
    const std::uint8_t *rgb,
    std::size_t size,
    unsigned int width,
    unsigned int height,
    int quality) {
  auto *encoder = static_cast<XGCSnapshotJpegHardwareEncoderV1 *>(
      encoderStorage_);
  if (!encoder) {
    return {{}, backend_, "hardware JPEG backend is not open"};
  }
  std::uint8_t *jpeg = nullptr;
  std::size_t jpegSize = 0;
  char error[512] = {};
  const int status = encoder->encode_rgb(
      encoder->context, rgb, size, width, height, quality, &jpeg, &jpegSize,
      error, sizeof(error));
  if (status != 0 || !jpeg || jpegSize < 4 || jpegSize > (32u << 20) ||
      jpeg[0] != 0xff || jpeg[1] != 0xd8 || jpeg[jpegSize - 2] != 0xff ||
      jpeg[jpegSize - 1] != 0xd9) {
    if (jpeg) {
      encoder->release_jpeg(encoder->context, jpeg);
    }
    return {
        {}, backend_,
        *error ? error : "hardware JPEG backend returned an invalid image"};
  }
  std::vector<std::uint8_t> bytes(jpeg, jpeg + jpegSize);
  encoder->release_jpeg(encoder->context, jpeg);
  return {std::move(bytes), backend_, ""};
}

std::optional<SnapshotJpegPolicy> ParseSnapshotJpegPolicy(
    const std::string &value) {
  if (value == "auto") {
    return SnapshotJpegPolicy::kAuto;
  }
  if (value == "hardware") {
    return SnapshotJpegPolicy::kHardware;
  }
  if (value == "cpu") {
    return SnapshotJpegPolicy::kCPU;
  }
  return std::nullopt;
}

const char *SnapshotJpegPolicyName(SnapshotJpegPolicy policy) {
  switch (policy) {
    case SnapshotJpegPolicy::kAuto:
      return "auto";
    case SnapshotJpegPolicy::kHardware:
      return "hardware";
    case SnapshotJpegPolicy::kCPU:
      return "cpu";
  }
  return "unknown";
}

SnapshotJpegBackendDecision SelectSnapshotJpegBackend(
    SnapshotJpegPolicy policy,
    bool hardwareAvailable,
    const std::string &hardwareBackend) {
  if (policy == SnapshotJpegPolicy::kCPU) {
    return {
        true, false, false, CPUBackendName(), "not-requested", ""};
  }
  if (hardwareAvailable) {
    return {
        true,
        true,
        policy == SnapshotJpegPolicy::kAuto,
        hardwareBackend,
        "available",
        ""};
  }
  if (policy == SnapshotJpegPolicy::kHardware) {
    return {
        false,
        false,
        false,
        "unavailable",
        "unavailable",
        "snapshot JPEG policy requires a hardware backend, but none passed runtime preflight"};
  }
  return {
      true,
      false,
      true,
      CPUBackendName(),
      "unavailable",
      ""};
}

SnapshotJpegEncodeResult EncodeSnapshotJpegCPU(
    const std::uint8_t *rgb,
    std::size_t size,
    unsigned int width,
    unsigned int height,
    int quality) {
  if (!rgb || width == 0 || height == 0 || width > 8192 || height > 8192 ||
      quality < 1 || quality > 100 ||
      width > std::numeric_limits<std::size_t>::max() / height / 3 ||
      size != static_cast<std::size_t>(width) * height * 3) {
    return {{}, CPUBackendName(), "snapshot RGB input is invalid"};
  }

  jpeg_compress_struct encoder{};
  JPEGErrorManager errors{};
  unsigned char *encoded = nullptr;
  unsigned long encodedSize = 0;
  bool created = false;
  encoder.err = jpeg_std_error(&errors.manager);
  errors.manager.error_exit = JPEGErrorExit;
  if (setjmp(errors.jumpBuffer) != 0) {
    if (created) {
      jpeg_destroy_compress(&encoder);
    }
    std::free(encoded);
    return {{}, CPUBackendName(), "libjpeg could not encode the snapshot"};
  }

  jpeg_create_compress(&encoder);
  created = true;
  jpeg_mem_dest(&encoder, &encoded, &encodedSize);
  encoder.image_width = width;
  encoder.image_height = height;
  encoder.input_components = 3;
  encoder.in_color_space = JCS_RGB;
  jpeg_set_defaults(&encoder);
  jpeg_set_quality(&encoder, quality, TRUE);
  jpeg_start_compress(&encoder, TRUE);
  while (encoder.next_scanline < encoder.image_height) {
    JSAMPROW row = const_cast<JSAMPLE *>(
        rgb + static_cast<std::size_t>(encoder.next_scanline) * width * 3);
    jpeg_write_scanlines(&encoder, &row, 1);
  }
  jpeg_finish_compress(&encoder);
  std::vector<std::uint8_t> result(encoded, encoded + encodedSize);
  jpeg_destroy_compress(&encoder);
  std::free(encoded);
  return {std::move(result), CPUBackendName(), ""};
}

bool CopyGazeboPBOToImageOrder(
    const std::uint8_t *renderTextureRows,
    std::size_t size,
    unsigned int width,
    unsigned int height,
    std::vector<std::uint8_t> *imageRows) {
  if (!renderTextureRows || !imageRows || width == 0 || height == 0 ||
      width > std::numeric_limits<std::size_t>::max() / height / 3 ||
      size != static_cast<std::size_t>(width) * height * 3) {
    return false;
  }
  imageRows->assign(renderTextureRows, renderTextureRows + size);
  return true;
}

}  // namespace gazebo_sim_camera
