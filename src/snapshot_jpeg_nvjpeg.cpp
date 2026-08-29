#include "snapshot_jpeg_hardware_abi.h"

#include <cuda_runtime_api.h>
#include <nvjpeg.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <string>

namespace {

struct NvjpegContext {
  int device = 0;
  nvjpegHandle_t handle = nullptr;
  nvjpegEncoderState_t state = nullptr;
  nvjpegEncoderParams_t params = nullptr;
  cudaStream_t stream = nullptr;
  unsigned char *deviceRGB = nullptr;
  std::size_t deviceCapacity = 0;
  std::mutex mutex;
};

void WriteError(char *destination, std::size_t size, const std::string &message) {
  if (!destination || size == 0) {
    return;
  }
  std::snprintf(destination, size, "%s", message.c_str());
}

std::string CUDAError(const char *operation, cudaError_t status) {
  return std::string(operation) + ": " + cudaGetErrorString(status);
}

std::string NvjpegError(const char *operation, nvjpegStatus_t status) {
  return std::string(operation) + " failed with nvJPEG status " +
      std::to_string(static_cast<int>(status));
}

void DestroyContext(NvjpegContext *context) {
  if (!context) {
    return;
  }
  cudaSetDevice(context->device);
  if (context->stream) {
    cudaStreamSynchronize(context->stream);
  }
  if (context->deviceRGB) {
    cudaFree(context->deviceRGB);
  }
  if (context->params) {
    nvjpegEncoderParamsDestroy(context->params);
  }
  if (context->state) {
    nvjpegEncoderStateDestroy(context->state);
  }
  if (context->handle) {
    nvjpegDestroy(context->handle);
  }
  if (context->stream) {
    cudaStreamDestroy(context->stream);
  }
  delete context;
}

void Destroy(void *opaque) {
  DestroyContext(static_cast<NvjpegContext *>(opaque));
}

void ReleaseJPEG(void *, std::uint8_t *jpeg) {
  std::free(jpeg);
}

int EncodeRGBImpl(
    void *opaque,
    const std::uint8_t *rgb,
    std::size_t rgbSize,
    std::uint32_t width,
    std::uint32_t height,
    int quality,
    std::uint8_t **jpeg,
    std::size_t *jpegSize,
    char *error,
    std::size_t errorSize) {
  auto *context = static_cast<NvjpegContext *>(opaque);
  if (!context || !rgb || !jpeg || !jpegSize || width < 16 || height < 16 ||
      width > 8192 || height > 8192 || quality < 1 || quality > 100 ||
      width > std::numeric_limits<std::size_t>::max() / height / 3 ||
      rgbSize != static_cast<std::size_t>(width) * height * 3) {
    WriteError(error, errorSize, "nvJPEG received invalid RGB input");
    return 1;
  }
  *jpeg = nullptr;
  *jpegSize = 0;
  std::lock_guard<std::mutex> guard(context->mutex);

  cudaError_t cudaStatus = cudaSetDevice(context->device);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("select CUDA device", cudaStatus));
    return 1;
  }

  if (context->deviceCapacity < rgbSize) {
    if (context->deviceRGB) {
      cudaFree(context->deviceRGB);
      context->deviceRGB = nullptr;
      context->deviceCapacity = 0;
    }
    const cudaError_t allocated = cudaMalloc(
        reinterpret_cast<void **>(&context->deviceRGB), rgbSize);
    if (allocated != cudaSuccess) {
      WriteError(error, errorSize, CUDAError("cudaMalloc RGB", allocated));
      return 1;
    }
    context->deviceCapacity = rgbSize;
  }
  cudaStatus = cudaMemcpyAsync(
      context->deviceRGB, rgb, rgbSize, cudaMemcpyHostToDevice,
      context->stream);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("cudaMemcpyAsync RGB", cudaStatus));
    return 1;
  }

  nvjpegStatus_t status = nvjpegEncoderParamsSetQuality(
      context->params, quality, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(context->stream);
    WriteError(error, errorSize, NvjpegError("set JPEG quality", status));
    return 1;
  }
  status = nvjpegEncoderParamsSetSamplingFactors(
      context->params, NVJPEG_CSS_420, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(context->stream);
    WriteError(error, errorSize, NvjpegError("set JPEG sampling", status));
    return 1;
  }

  nvjpegImage_t image{};
  image.channel[0] = context->deviceRGB;
  image.pitch[0] = static_cast<std::size_t>(width) * 3;
  status = nvjpegEncodeImage(
      context->handle, context->state, context->params, &image,
      NVJPEG_INPUT_RGBI, static_cast<int>(width), static_cast<int>(height),
      context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(context->stream);
    WriteError(error, errorSize, NvjpegError("encode RGB", status));
    return 1;
  }

  std::size_t length = 0;
  status = nvjpegEncodeRetrieveBitstream(
      context->handle, context->state, nullptr, &length, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(context->stream);
    WriteError(error, errorSize, NvjpegError("query bitstream size", status));
    return 1;
  }
  cudaStatus = cudaStreamSynchronize(context->stream);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("synchronize encode", cudaStatus));
    return 1;
  }
  if (length < 4 || length > (32u << 20)) {
    WriteError(
        error, errorSize, "nvJPEG returned an invalid bitstream size");
    return 1;
  }

  auto *output = static_cast<std::uint8_t *>(std::malloc(length));
  if (!output) {
    WriteError(error, errorSize, "allocate nvJPEG output failed");
    return 1;
  }
  std::size_t capacity = length;
  status = nvjpegEncodeRetrieveBitstream(
      context->handle, context->state, output, &capacity, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    cudaStreamSynchronize(context->stream);
    std::free(output);
    WriteError(error, errorSize, NvjpegError("retrieve bitstream", status));
    return 1;
  }
  cudaStatus = cudaStreamSynchronize(context->stream);
  if (cudaStatus != cudaSuccess) {
    std::free(output);
    WriteError(error, errorSize, CUDAError("synchronize bitstream", cudaStatus));
    return 1;
  }
  if (capacity < 4 || capacity > length || output[0] != 0xff ||
      output[1] != 0xd8 || output[capacity - 2] != 0xff ||
      output[capacity - 1] != 0xd9) {
    std::free(output);
    WriteError(error, errorSize, "nvJPEG returned invalid JPEG markers");
    return 1;
  }
  *jpeg = output;
  *jpegSize = capacity;
  return 0;
}

int EncodeRGB(
    void *opaque,
    const std::uint8_t *rgb,
    std::size_t rgbSize,
    std::uint32_t width,
    std::uint32_t height,
    int quality,
    std::uint8_t **jpeg,
    std::size_t *jpegSize,
    char *error,
    std::size_t errorSize) noexcept {
  try {
    return EncodeRGBImpl(
        opaque, rgb, rgbSize, width, height, quality, jpeg, jpegSize, error,
        errorSize);
  } catch (const std::exception &failure) {
    if (jpeg) {
      *jpeg = nullptr;
    }
    if (jpegSize) {
      *jpegSize = 0;
    }
    WriteError(error, errorSize, failure.what());
    return 1;
  } catch (...) {
    if (jpeg) {
      *jpeg = nullptr;
    }
    if (jpegSize) {
      *jpegSize = 0;
    }
    WriteError(error, errorSize, "unexpected nvJPEG backend exception");
    return 1;
  }
}

}  // namespace

extern "C" int xgc_snapshot_jpeg_hardware_create_v1(
    std::uint32_t requestedAbi,
    XGCSnapshotJpegHardwareEncoderV1 *encoder,
    char *error,
    std::size_t errorSize) {
  try {
  if (!encoder || requestedAbi != XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1) {
    WriteError(error, errorSize, "unsupported snapshot JPEG hardware ABI");
    return 1;
  }
  int deviceCount = 0;
  cudaError_t cudaStatus = cudaGetDeviceCount(&deviceCount);
  if (cudaStatus != cudaSuccess || deviceCount < 1) {
    WriteError(
        error, errorSize,
        cudaStatus == cudaSuccess
            ? "no CUDA device is available"
            : CUDAError("enumerate CUDA devices", cudaStatus));
    return 1;
  }

  auto *context = new (std::nothrow) NvjpegContext();
  if (!context) {
    WriteError(error, errorSize, "allocate nvJPEG context failed");
    return 1;
  }
  cudaStatus = cudaGetDevice(&context->device);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("select current CUDA device", cudaStatus));
    DestroyContext(context);
    return 1;
  }
  cudaStatus = cudaStreamCreateWithFlags(
      &context->stream, cudaStreamNonBlocking);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("create CUDA stream", cudaStatus));
    DestroyContext(context);
    return 1;
  }
  nvjpegStatus_t status = nvjpegCreateSimple(&context->handle);
  if (status != NVJPEG_STATUS_SUCCESS) {
    WriteError(error, errorSize, NvjpegError("create nvJPEG handle", status));
    DestroyContext(context);
    return 1;
  }
  status = nvjpegEncoderStateCreate(
      context->handle, &context->state, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    WriteError(error, errorSize, NvjpegError("create encoder state", status));
    DestroyContext(context);
    return 1;
  }
  status = nvjpegEncoderParamsCreate(
      context->handle, &context->params, context->stream);
  if (status != NVJPEG_STATUS_SUCCESS) {
    WriteError(error, errorSize, NvjpegError("create encoder params", status));
    DestroyContext(context);
    return 1;
  }
  status = nvjpegEncoderParamsSetEncoding(
      context->params, NVJPEG_ENCODING_BASELINE_DCT, context->stream);
  if (status == NVJPEG_STATUS_SUCCESS) {
    status = nvjpegEncoderParamsSetOptimizedHuffman(
        context->params, 0, context->stream);
  }
  if (status == NVJPEG_STATUS_SUCCESS) {
    status = nvjpegEncoderParamsSetSamplingFactors(
        context->params, NVJPEG_CSS_420, context->stream);
  }
  if (status != NVJPEG_STATUS_SUCCESS) {
    WriteError(error, errorSize, NvjpegError("configure baseline encoder", status));
    DestroyContext(context);
    return 1;
  }
  cudaStatus = cudaStreamSynchronize(context->stream);
  if (cudaStatus != cudaSuccess) {
    WriteError(error, errorSize, CUDAError("preflight encoder params", cudaStatus));
    DestroyContext(context);
    return 1;
  }

  // Factory success means a real encode/retrieve completed on the selected
  // device, not merely that shared libraries and opaque handles exist.
  std::uint8_t probeRGB[16 * 16 * 3] = {};
  std::uint8_t *probeJPEG = nullptr;
  std::size_t probeSize = 0;
  if (EncodeRGBImpl(
          context, probeRGB, sizeof(probeRGB), 16, 16, 90, &probeJPEG,
          &probeSize, error, errorSize) != 0) {
    DestroyContext(context);
    return 1;
  }
  std::free(probeJPEG);

  *encoder = {};
  encoder->abi_version = XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1;
  encoder->struct_size = sizeof(*encoder);
  encoder->backend_name = "nvjpeg-cuda";
  encoder->context = context;
  encoder->encode_rgb = EncodeRGB;
  encoder->release_jpeg = ReleaseJPEG;
  encoder->destroy = Destroy;
  return 0;
  } catch (const std::exception &failure) {
    WriteError(error, errorSize, failure.what());
    return 1;
  } catch (...) {
    WriteError(error, errorSize, "unexpected nvJPEG factory exception");
    return 1;
  }
}
