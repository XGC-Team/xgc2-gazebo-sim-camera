#include "snapshot_jpeg_hardware_abi.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

int Encode(
    void *, const uint8_t *, size_t, uint32_t, uint32_t, int,
    uint8_t **jpeg, size_t *jpegSize, char *, size_t) {
  static const uint8_t payload[] = {0xff, 0xd8, 'x', 'g', 'c', 0xff, 0xd9};
  *jpeg = static_cast<uint8_t *>(std::malloc(sizeof(payload)));
  if (!*jpeg) {
    return 1;
  }
  std::memcpy(*jpeg, payload, sizeof(payload));
  *jpegSize = sizeof(payload);
  return 0;
}

void Release(void *, uint8_t *jpeg) {
  std::free(jpeg);
}

void Destroy(void *) {
}

}  // namespace

extern "C" int xgc_snapshot_jpeg_hardware_create_v1(
    uint32_t requestedAbi,
    XGCSnapshotJpegHardwareEncoderV1 *encoder,
    char *error,
    size_t errorSize) {
  if (!encoder || requestedAbi != XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1) {
    if (error && errorSize) {
      std::snprintf(error, errorSize, "unsupported ABI");
    }
    return 1;
  }
  *encoder = {};
  encoder->abi_version = XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1;
  encoder->struct_size = sizeof(*encoder);
  encoder->backend_name = "fake-hardware-jpeg";
  encoder->encode_rgb = Encode;
  encoder->release_jpeg = Release;
  encoder->destroy = Destroy;
  return 0;
}
