#include "lvgl.h"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
#define LV_ATTRIBUTE_MEM_ALIGN
#endif

#ifndef LV_ATTRIBUTE_IMG_BRAKE
#define LV_ATTRIBUTE_IMG_BRAKE
#endif

const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_IMG_BRAKE uint8_t brake_map[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x0b, 0x70, 0x00, 0x02, 0x7b, 0xdd, 0xb7, 0x20, 0x00, 0x07, 0xb0, 0x00, 
  0x00, 0x9f, 0xe1, 0x01, 0xaf, 0xff, 0xff, 0xff, 0xf9, 0x10, 0x1e, 0xf9, 0x00, 
  0x03, 0xff, 0x30, 0x2d, 0xff, 0xa5, 0x33, 0x5a, 0xff, 0xd2, 0x03, 0xff, 0x30, 
  0x0b, 0xf8, 0x01, 0xdf, 0xc2, 0x00, 0x00, 0x00, 0x2d, 0xfd, 0x10, 0x8f, 0xb0, 
  0x2f, 0xf1, 0x0a, 0xfc, 0x00, 0x00, 0xcc, 0x00, 0x00, 0xcf, 0xa0, 0x1f, 0xf2, 
  0x7f, 0xa0, 0x2f, 0xf2, 0x00, 0x00, 0xff, 0x00, 0x00, 0x2f, 0xf2, 0x0a, 0xf7, 
  0xbf, 0x50, 0x8f, 0x90, 0x00, 0x00, 0xff, 0x00, 0x00, 0x09, 0xf8, 0x05, 0xfb, 
  0xdf, 0x20, 0xcf, 0x40, 0x00, 0x00, 0xff, 0x00, 0x00, 0x04, 0xfc, 0x02, 0xfd, 
  0xff, 0x10, 0xef, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0x02, 0xfe, 0x01, 0xff, 
  0xff, 0x10, 0xef, 0x20, 0x00, 0x00, 0xff, 0x00, 0x00, 0x02, 0xfe, 0x01, 0xff, 
  0xdf, 0x20, 0xcf, 0x40, 0x00, 0x00, 0x55, 0x00, 0x00, 0x04, 0xfc, 0x02, 0xfd, 
  0xbf, 0x50, 0x8f, 0x90, 0x00, 0x00, 0x22, 0x00, 0x00, 0x09, 0xf8, 0x05, 0xfb, 
  0x7f, 0xa0, 0x3f, 0xf1, 0x00, 0x00, 0xff, 0x00, 0x00, 0x1f, 0xf3, 0x0a, 0xf7, 
  0x2f, 0xf1, 0x0b, 0xfb, 0x00, 0x00, 0xee, 0x00, 0x00, 0xbf, 0xb0, 0x1f, 0xf2, 
  0x0b, 0xf8, 0x01, 0xef, 0xb1, 0x00, 0x00, 0x00, 0x1b, 0xfe, 0x10, 0x8f, 0xb0, 
  0x03, 0xff, 0x30, 0x3e, 0xff, 0x83, 0x11, 0x38, 0xff, 0xe3, 0x03, 0xff, 0x30, 
  0x00, 0x9f, 0xe1, 0x02, 0xbf, 0xff, 0xff, 0xff, 0xfb, 0x10, 0x1e, 0xf9, 0x00, 
  0x00, 0x0b, 0x80, 0x00, 0x03, 0x9d, 0xff, 0xd9, 0x30, 0x00, 0x08, 0xb0, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const lv_img_dsc_t brake = {
  .header.always_zero = 0,
  .header.w = 26,
  .header.h = 26,
  .data_size = 338,
  .header.cf = LV_IMG_CF_ALPHA_4BIT,
  .data = brake_map,
};