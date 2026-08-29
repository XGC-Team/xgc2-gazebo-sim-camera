#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XGC_SNAPSHOT_JPEG_HARDWARE_ABI_V1 1u
#define XGC_SNAPSHOT_JPEG_HARDWARE_CREATE_SYMBOL_V1 \
  "xgc_snapshot_jpeg_hardware_create_v1"

typedef struct XGCSnapshotJpegHardwareEncoderV1 {
  uint32_t abi_version;
  size_t struct_size;
  const char *backend_name;
  void *context;
  int (*encode_rgb)(
      void *context,
      const uint8_t *rgb,
      size_t rgb_size,
      uint32_t width,
      uint32_t height,
      int quality,
      uint8_t **jpeg,
      size_t *jpeg_size,
      char *error,
      size_t error_size);
  void (*release_jpeg)(void *context, uint8_t *jpeg);
  void (*destroy)(void *context);
} XGCSnapshotJpegHardwareEncoderV1;

typedef int (*XGCSnapshotJpegHardwareCreateV1)(
    uint32_t requested_abi,
    XGCSnapshotJpegHardwareEncoderV1 *encoder,
    char *error,
    size_t error_size);

#ifdef __cplusplus
}
#endif
