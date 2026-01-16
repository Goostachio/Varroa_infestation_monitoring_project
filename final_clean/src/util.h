#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

inline void bgr_to_rgb_inplace(uint8_t* buf, size_t pixels) {
  for (size_t i = 0; i < pixels; ++i) {
    uint8_t* p = &buf[i * 3];
    uint8_t t = p[0];
    p[0] = p[2];
    p[2] = t;
  }
}

inline void sanitize_label(const char* in, char out[12]) {
  memset(out, 0, 12);
  uint8_t j=0;
  for (const char* p=in; *p && j<11; ++p) {
    char c=*p;
    if ((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')) out[j++]=c;
    else if (c==' '||c=='-'||c=='.') out[j++]='_';
  }
  if (j==0) strncpy(out,"obj",11);
}

inline void basename_no_ext_v(const char* path, char* out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  out[0] = 0;
  const char* base = strrchr(path, '/');
  base = base ? (base + 1) : path;
  const char* dot = strrchr(base, '.');
  size_t n = dot ? (size_t)(dot - base) : strlen(base);
  if (n >= out_sz) n = out_sz - 1;
  memcpy(out, base, n);
  out[n] = 0;
}

inline void basename_with_ext_v(const char* path, char* out, size_t out_sz) {
  if (!out || out_sz == 0) return;
  out[0] = 0;
  const char* base = strrchr(path, '/');
  base = base ? (base + 1) : path;
  size_t n = strlen(base);
  if (n >= out_sz) n = out_sz - 1;
  memcpy(out, base, n);
  out[n] = 0;
}

inline bool jpeg_get_dims_v(const uint8_t* data, size_t len, int& w, int& h) {
  w = h = 0;
  if (len < 4 || data[0]!=0xFF || data[1]!=0xD8) return false;
  size_t i = 2;
  while (i + 8 < len) {
    if (data[i] != 0xFF) { i++; continue; }
    uint8_t m = data[i+1];
    if (m == 0xD9 || m == 0xDA) break;
    uint16_t seglen = (i+3 < len) ? ((data[i+2] << 8) | data[i+3]) : 0;
    if (seglen < 2 || i + 2 + seglen > len) break;
    if (m == 0xC0 || m == 0xC2) {
      if (seglen >= 7) {
        h = (data[i+5] << 8) | data[i+6];
        w = (data[i+7] << 8) | data[i+8];
        return (w > 0 && h > 0);
      }
    }
    i += 2 + seglen;
  }
  return false;
}

inline void ei_calc_crop_map(int src_w, int src_h, int dst_w, int dst_h,
                             int &crop_x, int &crop_y, int &crop_w, int &crop_h,
                             float &scale_x, float &scale_y) {
  if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0) {
    crop_x = 0; crop_y = 0; crop_w = src_w; crop_h = src_h;
    scale_x = (dst_w > 0) ? (float)src_w / (float)dst_w : 1.0f;
    scale_y = (dst_h > 0) ? (float)src_h / (float)dst_h : 1.0f;
    return;
  }

  const float src_ar = (float)src_w / (float)src_h;
  const float dst_ar = (float)dst_w / (float)dst_h;

  if (src_ar > dst_ar) {
    crop_h = src_h;
    crop_w = (int)lrintf((float)src_h * dst_ar);
    if (crop_w < 1) crop_w = 1;
    if (crop_w > src_w) crop_w = src_w;
    crop_x = (src_w - crop_w) / 2;
    crop_y = 0;
  } else {
    crop_w = src_w;
    crop_h = (int)lrintf((float)src_w / dst_ar);
    if (crop_h < 1) crop_h = 1;
    if (crop_h > src_h) crop_h = src_h;
    crop_x = 0;
    crop_y = (src_h - crop_h) / 2;
  }

  scale_x = (float)crop_w / (float)dst_w;
  scale_y = (float)crop_h / (float)dst_h;
}
