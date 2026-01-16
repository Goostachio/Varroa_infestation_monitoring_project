#include "ei_signal_shim.h"
#include "../globals.h"

int ei_bee_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i=0;i<length;i++, pixel_ix+=3) {
    out_ptr[i] = (snapshot_buf[pixel_ix] << 16) |
                 (snapshot_buf[pixel_ix + 1] << 8) |
                  snapshot_buf[pixel_ix + 2];
  }
  return 0;
}

int ei_varroa_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;
  for (size_t i = 0; i < length; i++, pixel_ix += 3) {
    out_ptr[i] = (float)((uint32_t)g_var_snapshot_buf[pixel_ix] << 16 |
                         (uint32_t)g_var_snapshot_buf[pixel_ix + 1] << 8  |
                         (uint32_t)g_var_snapshot_buf[pixel_ix + 2]);
  }
  return 0;
}
