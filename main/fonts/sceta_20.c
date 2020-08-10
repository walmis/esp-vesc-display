#include "lvgl.h"

/*******************************************************************************
 * Size: 16 px
 * Bpp: 2
 * Opts: 
 ******************************************************************************/

#ifndef SCETA_20
#define SCETA_20 1
#endif

#if SCETA_20

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t gylph_bitmap[] = {
    /* U+30 "0" */
    0xf, 0xf2, 0xaf, 0xff, 0xe8, 0x4a, 0xf, 0xfe,
    0x2d, 0xea, 0x21, 0xff, 0xd0, 0x69, 0xe8, 0x3f,
    0xe6, 0xd0, 0xff, 0xed, 0x3c, 0x87, 0xcc, 0x87,
    0xff, 0x79, 0x90, 0xe7, 0xf, 0xfe, 0x3, 0x7f,
    0xff, 0x1a, 0x43, 0xff, 0x81, 0xa1, 0xff, 0xc5,
    0xa2, 0x1f, 0xfc, 0x66, 0x81, 0xff, 0xd1, 0x43,
    0xff, 0x98, 0x87, 0xff, 0xfc, 0x3f, 0xff, 0xe1,
    0xff, 0xe4, 0x6f, 0xf4, 0x87, 0xff, 0x7e, 0x88,
    0x79, 0xa0, 0x7f, 0xff, 0xc3, 0xff, 0x93, 0x44,
    0x3c, 0xd0, 0x3f, 0xfb, 0xed, 0xfe, 0x90, 0xff,
    0xff, 0x87, 0xff, 0xfc, 0x3f, 0xfc, 0x88, 0x7f,
    0xf3, 0x10, 0xff, 0xe8, 0xd1, 0xf, 0xfe, 0x33,
    0x40, 0xff, 0xe2, 0xb8, 0x7f, 0xf0, 0x1b, 0xff,
    0xf8, 0xd2, 0x1f, 0xfc, 0xd, 0xe, 0x64, 0x3f,
    0xfb, 0xcc, 0x87, 0xcd, 0xa1, 0xff, 0xda, 0x79,
    0xf, 0xfa, 0xf5, 0x10, 0xff, 0xe8, 0x34, 0xf4,
    0x1f, 0x0,

    /* U+31 "1" */
    0xf, 0xfe, 0x62, 0xd5, 0x7a, 0x43, 0xff, 0xa8,
    0xb5, 0x5f, 0xea, 0x96, 0xf, 0xfe, 0x7b, 0x4f,
    0xf5, 0x4b, 0x7, 0xff, 0x69, 0xd4, 0x1f, 0xff,
    0xf0, 0xff, 0xed, 0xad, 0x57, 0xa4, 0x3f, 0xfa,
    0xcf, 0x4f, 0xf5, 0x4b, 0x7, 0xff, 0x6d, 0x60,
    0xff, 0xff, 0x87, 0xff, 0xfc, 0x3f, 0xff, 0xe1,
    0xff, 0xff, 0xf, 0xff, 0xf8, 0x7f, 0xff, 0xc3,
    0xff, 0xfe, 0x1f, 0xff, 0xf0, 0xff, 0xff, 0x87,
    0xff, 0x2d, 0xbf, 0xff, 0x89, 0x21, 0xfe, 0x6f,
    0xff, 0xe2, 0x48, 0x7f, 0xff, 0xc3, 0xff, 0xfe,
    0x1f, 0xfc, 0x20,

    /* U+32 "2" */
    0x3, 0x7f, 0xff, 0x66, 0x50, 0x7f, 0xf9, 0xda,
    0x7a, 0xf, 0xff, 0x4b, 0xc8, 0x7f, 0xfa, 0x19,
    0xe, 0x6f, 0xff, 0xea, 0x48, 0x7f, 0xf0, 0x34,
    0x3f, 0xfb, 0x4d, 0x3, 0xff, 0xd4, 0x87, 0xff,
    0xfc, 0x3f, 0xfa, 0x88, 0x7f, 0xf9, 0x9a, 0x7,
    0xff, 0x35, 0x57, 0xff, 0xf2, 0xe4, 0x3f, 0xf8,
    0x1a, 0x1f, 0xd7, 0xa8, 0x87, 0xff, 0x5d, 0x90,
    0xf9, 0xb4, 0x3f, 0xfb, 0x4f, 0x21, 0xf3, 0x21,
    0xff, 0xd7, 0x69, 0xe8, 0x3f, 0x9c, 0x3f, 0xf8,
    0xd, 0xff, 0xfc, 0xb9, 0x41, 0xff, 0xcd, 0xa2,
    0x1f, 0xfe, 0x64, 0x3f, 0xff, 0xe1, 0xff, 0xff,
    0xf, 0xff, 0xab, 0x7f, 0xff, 0x5a, 0x43, 0xff,
    0xfe, 0x1f, 0xff, 0xf0, 0xff, 0xe1, 0x0,

    /* U+33 "3" */
    0x3, 0x7f, 0xff, 0x66, 0x50, 0x7f, 0xf9, 0xda,
    0x7a, 0xf, 0xff, 0x4b, 0xc8, 0x7f, 0xfa, 0x19,
    0xe, 0x6f, 0xff, 0xea, 0x48, 0x7f, 0xf0, 0x34,
    0x3f, 0xfb, 0x4d, 0x3, 0xff, 0xd4, 0x87, 0xff,
    0xfc, 0x3f, 0xfa, 0x88, 0x7f, 0xf9, 0x9a, 0x7,
    0xff, 0x45, 0xbf, 0xff, 0x91, 0x21, 0xff, 0xc1,
    0x43, 0xff, 0xcc, 0xd0, 0x3f, 0xff, 0xe1, 0xff,
    0xd2, 0x68, 0x1f, 0xfc, 0x56, 0xff, 0xfe, 0x44,
    0x87, 0xff, 0x5, 0xf, 0xfe, 0xd3, 0x40, 0xff,
    0xf5, 0x21, 0xff, 0xff, 0xf, 0xfe, 0xa2, 0x1f,
    0xfe, 0x66, 0x81, 0xff, 0xc5, 0x6f, 0xff, 0xea,
    0x48, 0x7f, 0xf0, 0x34, 0x3f, 0xfc, 0xcc, 0x87,
    0xff, 0x95, 0xe4, 0x3f, 0xfc, 0x2d, 0x3d, 0x7,
    0xc0,

    /* U+34 "4" */
    0xf, 0xfe, 0xbb, 0x7f, 0xfa, 0x43, 0xff, 0xe8,
    0xff, 0xfd, 0x7, 0xff, 0x65, 0xc3, 0xfc, 0xe1,
    0xff, 0xd8, 0x74, 0x3f, 0x9d, 0xf, 0xfe, 0xc6,
    0x87, 0xfb, 0x43, 0xff, 0xb0, 0xa0, 0xff, 0x28,
    0x3f, 0xfb, 0xe, 0x1f, 0xe7, 0xf, 0xfe, 0xc3,
    0xa1, 0xfc, 0xe8, 0x7f, 0xf6, 0x34, 0x3f, 0xda,
    0x1f, 0xfd, 0x85, 0x7, 0xfd, 0x7f, 0xff, 0x12,
    0x43, 0xff, 0x8c, 0xe1, 0xff, 0xe4, 0x74, 0x3f,
    0xfc, 0x9a, 0x1f, 0xfe, 0x5b, 0xff, 0xfa, 0x72,
    0x1f, 0xff, 0xf0, 0xff, 0xff, 0x87, 0xff, 0xfc,
    0x3f, 0xff, 0xe1, 0xff, 0xff, 0xf, 0xff, 0xf8,
    0x7f, 0xf2, 0xc0,

    /* U+35 "5" */
    0x3, 0x7f, 0xff, 0x8a, 0x43, 0xff, 0xfe, 0x1f,
    0xff, 0xf0, 0xff, 0xe8, 0x37, 0xff, 0xf5, 0xa4,
    0x3f, 0xff, 0xe1, 0xff, 0xff, 0xf, 0xff, 0xf8,
    0x7f, 0xfc, 0x5b, 0xff, 0xf9, 0xb2, 0x83, 0xff,
    0xce, 0xd3, 0xd0, 0x7f, 0xfa, 0x5e, 0x43, 0xff,
    0xd0, 0xc8, 0x73, 0x7f, 0xff, 0x52, 0x43, 0xff,
    0x81, 0xa1, 0xff, 0xda, 0x68, 0x1f, 0xfe, 0xa4,
    0x3f, 0xff, 0xe1, 0xff, 0xd4, 0x43, 0xff, 0xcc,
    0xd0, 0x3f, 0xf8, 0xad, 0xff, 0xfd, 0x49, 0xf,
    0xfe, 0x6, 0x87, 0xff, 0x99, 0x90, 0xff, 0xf2,
    0xbc, 0x87, 0xff, 0x85, 0xa7, 0xa0, 0xf8,

    /* U+36 "6" */
    0xf, 0xf2, 0xaf, 0xff, 0xe8, 0x4a, 0xf, 0xfe,
    0x2d, 0xea, 0x21, 0xff, 0xd0, 0x69, 0xe8, 0x3f,
    0xe6, 0xd0, 0xff, 0xed, 0x3c, 0x87, 0xcc, 0x87,
    0xff, 0x79, 0x90, 0xe7, 0xf, 0xfe, 0x3, 0x7f,
    0xff, 0x1a, 0x43, 0xff, 0x81, 0xa1, 0xff, 0xc5,
    0xa2, 0x1f, 0xfc, 0x66, 0x81, 0xff, 0xd1, 0x43,
    0xff, 0x96, 0xff, 0xfd, 0x21, 0xff, 0xff, 0xf,
    0xff, 0xf8, 0x7f, 0xf4, 0x9b, 0xff, 0xf9, 0x72,
    0x83, 0xff, 0x98, 0xd1, 0xf, 0xfe, 0x5b, 0x4f,
    0x41, 0xff, 0xe9, 0x79, 0xf, 0xff, 0x43, 0x21,
    0xff, 0xc7, 0x6f, 0xff, 0xe3, 0x48, 0x7f, 0xf0,
    0x34, 0x3f, 0xf8, 0xb4, 0x43, 0xff, 0x8c, 0xd0,
    0x3f, 0xfa, 0x28, 0x7f, 0xf3, 0x10, 0xff, 0xff,
    0x87, 0xc8, 0x7f, 0xf3, 0x10, 0xff, 0xe8, 0xd1,
    0xf, 0xfe, 0x33, 0x40, 0xff, 0xe2, 0xb8, 0x7f,
    0xf0, 0x1b, 0xff, 0xf8, 0xd2, 0x1f, 0xfc, 0xd,
    0xe, 0x64, 0x3f, 0xfb, 0xcc, 0x87, 0xcd, 0xa1,
    0xff, 0xda, 0x79, 0xf, 0xfa, 0xf5, 0x10, 0xff,
    0xe8, 0x34, 0xf4, 0x1f, 0x0,

    /* U+37 "7" */
    0x3, 0x7f, 0xff, 0x8a, 0x43, 0xff, 0xfe, 0x1f,
    0xff, 0xf0, 0xff, 0xe2, 0x37, 0xff, 0xf5, 0xa0,
    0xff, 0xe0, 0x21, 0xff, 0xdb, 0x70, 0xff, 0x38,
    0x7f, 0xf6, 0xdd, 0xf, 0xe7, 0x43, 0xff, 0xb7,
    0xa1, 0xfe, 0xd0, 0xff, 0xed, 0xa8, 0x3f, 0xca,
    0xf, 0xfe, 0xdb, 0x87, 0xf9, 0xc3, 0xff, 0xb6,
    0xe8, 0x7f, 0x3a, 0x1f, 0xfd, 0xbd, 0xf, 0xf6,
    0x87, 0xff, 0x6d, 0x41, 0xfe, 0x50, 0x7f, 0xf6,
    0xdc, 0x3f, 0xce, 0x1f, 0xfd, 0xb7, 0x43, 0xf9,
    0xd0, 0xff, 0xed, 0xe8, 0x7f, 0xb4, 0x3f, 0xfb,
    0x6a, 0xf, 0xf2, 0x83, 0xff, 0xb6, 0xe1, 0xfe,
    0x70, 0xff, 0xed, 0xba, 0x1f, 0xce, 0x87, 0xff,
    0x6f, 0x43, 0xfd, 0xa1, 0xff, 0xdb, 0x50, 0x7f,
    0x94, 0x1f, 0xfd, 0xb7, 0xf, 0xf3, 0x87, 0xff,
    0x6d, 0xd0, 0xfe, 0x74, 0x3f, 0xfb, 0x7a, 0x1f,
    0xed, 0xf, 0xfe, 0x20,

    /* U+38 "8" */
    0xf, 0xf2, 0xaf, 0xff, 0xe8, 0x4a, 0xf, 0xfe,
    0x2d, 0xea, 0x21, 0xff, 0xd0, 0x69, 0xe8, 0x3f,
    0xe6, 0xd0, 0xff, 0xed, 0x3c, 0x87, 0xcc, 0x87,
    0xff, 0x79, 0x90, 0xe7, 0xf, 0xfe, 0x3, 0x7f,
    0xff, 0x1a, 0x43, 0xff, 0x81, 0xa1, 0xff, 0xc5,
    0xa2, 0x1f, 0xfc, 0x66, 0x81, 0xff, 0xd1, 0x43,
    0xff, 0x98, 0x87, 0xff, 0xfc, 0x3e, 0x43, 0xff,
    0x98, 0x87, 0xff, 0x46, 0x88, 0x7f, 0xf1, 0x9a,
    0x7, 0xff, 0x15, 0xf, 0xfe, 0xb, 0x7f, 0xff,
    0x1a, 0x43, 0xff, 0x82, 0x87, 0x51, 0xf, 0xfe,
    0xf3, 0x40, 0xff, 0xf8, 0x51, 0xf, 0xfe, 0xf3,
    0x40, 0xe4, 0x3f, 0xf8, 0x2d, 0xff, 0xfc, 0x69,
    0xf, 0xfe, 0xa, 0x1f, 0xfc, 0x5a, 0x21, 0xff,
    0xc6, 0x68, 0x1f, 0xfd, 0x14, 0x3f, 0xf9, 0x88,
    0x7f, 0xff, 0xc3, 0xe4, 0x3f, 0xf9, 0x88, 0x7f,
    0xf4, 0x68, 0x87, 0xff, 0x19, 0xa0, 0x7f, 0xf1,
    0x5c, 0x3f, 0xf8, 0xd, 0xff, 0xfc, 0x69, 0xf,
    0xfe, 0x6, 0x87, 0x32, 0x1f, 0xfd, 0xe6, 0x43,
    0xe6, 0xd0, 0xff, 0xed, 0x3c, 0x87, 0xfd, 0x7a,
    0x88, 0x7f, 0xf4, 0x1a, 0x7a, 0xf, 0x80,

    /* U+39 "9" */
    0xf, 0xf2, 0xaf, 0xff, 0xe8, 0x4a, 0xf, 0xfe,
    0x2d, 0xea, 0x21, 0xff, 0xd0, 0x69, 0xe8, 0x3f,
    0xe6, 0xd0, 0xff, 0xed, 0x3c, 0x87, 0xcc, 0x87,
    0xff, 0x79, 0x90, 0xe7, 0xf, 0xfe, 0x3, 0x7f,
    0xff, 0x1a, 0x43, 0xff, 0x81, 0xa1, 0xff, 0xc5,
    0xa2, 0x1f, 0xfc, 0x66, 0x81, 0xff, 0xd1, 0x43,
    0xff, 0x98, 0x87, 0xff, 0xfc, 0x3e, 0x43, 0xff,
    0x98, 0x87, 0xff, 0x46, 0x88, 0x7f, 0xf1, 0x9a,
    0x7, 0xff, 0x15, 0xc3, 0xff, 0x80, 0xdf, 0xff,
    0xc6, 0x90, 0xff, 0xe3, 0xb2, 0x1f, 0xfe, 0x86,
    0xd0, 0xff, 0xf4, 0xde, 0xa2, 0x1f, 0xfc, 0xb6,
    0x88, 0x7f, 0xf3, 0x15, 0x7f, 0xff, 0x2e, 0x43,
    0xff, 0xfe, 0x1f, 0xff, 0xf0, 0xff, 0xe9, 0x37,
    0xff, 0xc8, 0x7f, 0xf2, 0xd0, 0xff, 0xe8, 0xd1,
    0xf, 0xfe, 0x33, 0x40, 0xff, 0xe2, 0xb8, 0x7f,
    0xf0, 0x1b, 0xff, 0xf8, 0xd2, 0x1f, 0xfc, 0xd,
    0xe, 0x64, 0x3f, 0xfb, 0xcc, 0x87, 0xcd, 0xa1,
    0xff, 0xda, 0x79, 0xf, 0xfa, 0xf5, 0x10, 0xff,
    0xe8, 0x34, 0xf4, 0x1f, 0x0,

    /* U+41 "A" */
    0xf, 0xfe, 0x13, 0xff, 0xfc, 0xc4, 0x3f, 0xf9,
    0xee, 0x1f, 0xfc, 0xcd, 0xf, 0xfe, 0x6b, 0xa1,
    0xff, 0xcc, 0x74, 0x3f, 0xf9, 0x9a, 0x1f, 0xfc,
    0xe7, 0xf, 0xfe, 0x5a, 0x83, 0xfc, 0xff, 0x90,
    0xff, 0x28, 0x3f, 0xf9, 0x2e, 0x1f, 0xe7, 0xf,
    0x68, 0x7f, 0xb4, 0x3f, 0xf8, 0xee, 0x87, 0xf3,
    0xa1, 0xe7, 0x43, 0xf9, 0xd0, 0xff, 0xe3, 0x68,
    0x7f, 0xb4, 0x3f, 0x38, 0x7f, 0x9c, 0x3f, 0xf8,
    0xaa, 0xf, 0xf2, 0x83, 0xfc, 0xa0, 0xff, 0x28,
    0x3f, 0xf8, 0x6e, 0x1f, 0xe7, 0xf, 0xfe, 0x6,
    0x87, 0xfb, 0x43, 0xff, 0x82, 0xe8, 0x7f, 0x3a,
    0x1f, 0xfc, 0x7, 0x43, 0xf9, 0xd0, 0xff, 0xe0,
    0x68, 0x7f, 0xb4, 0x3f, 0xf8, 0x4e, 0x1f, 0xe7,
    0xf, 0xf9, 0x41, 0xff, 0x5f, 0xff, 0xc4, 0x83,
    0xfe, 0x50, 0x7f, 0x38, 0x7f, 0xf7, 0xb4, 0x3e,
    0x74, 0x3f, 0xfb, 0xce, 0x87, 0xb4, 0x3f, 0xfc,
    0xe, 0x1c, 0x87, 0xfc, 0xdf, 0xff, 0xca, 0x90,
    0xff, 0x90, 0xff, 0xff, 0x87, 0xff, 0xfc, 0x3f,
    0xff, 0xe1, 0xff, 0xff, 0xf, 0xff, 0xf8, 0x70,

    /* U+57 "W" */
    0x3, 0x7f, 0xfa, 0x43, 0x37, 0xff, 0xa4, 0x33,
    0x7f, 0xfa, 0x43, 0xff, 0xfe, 0x1f, 0xff, 0xf0,
    0xff, 0xff, 0x87, 0xff, 0xfc, 0x3f, 0xff, 0xe1,
    0xff, 0xff, 0xf, 0xff, 0xf8, 0x7f, 0xff, 0xc3,
    0xff, 0xfe, 0x1f, 0xff, 0xf0, 0xff, 0xfa, 0x21,
    0xff, 0x21, 0xff, 0xcc, 0x43, 0xfe, 0x43, 0xb4,
    0x3f, 0xda, 0x1f, 0xfc, 0x87, 0xf, 0xf3, 0x87,
    0x9d, 0xf, 0xe7, 0x43, 0xff, 0x8c, 0xe8, 0x7f,
    0x3a, 0x1f, 0x38, 0x7f, 0x9c, 0x3f, 0xf8, 0xda,
    0x1f, 0xed, 0xf, 0xe5, 0x7, 0xf9, 0xf4, 0x87,
    0xf9, 0xbc, 0x87, 0xf9, 0x41, 0xff, 0x68, 0x7f,
    0xf1, 0x5a, 0x21, 0xff, 0xc5, 0x70, 0xff, 0xe0,
    0x3a, 0x1f, 0xfc, 0x37, 0x56, 0x87, 0xff, 0xd,
    0xd0, 0xff, 0xe0, 0xb8, 0x7f, 0xf0, 0xf4, 0xe,
    0x1f, 0xfc, 0x3d, 0xf, 0x80
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 130, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 213, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 316, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 421, .adv_w = 288, .box_w = 51, .box_h = 24, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 512, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 599, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 748, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 872, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1031, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1180, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0},
    {.bitmap_index = 1332, .adv_w = 288, .box_w = 54, .box_h = 24, .ofs_x = -1, .ofs_y = 0}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/

static const uint16_t unicode_list_1[] = {
    0x0, 0x16
};

/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 48, .range_length = 10, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    },
    {
        .range_start = 65, .range_length = 23, .glyph_id_start = 11,
        .unicode_list = unicode_list_1, .glyph_id_ofs_list = NULL, .list_length = 2, .type = LV_FONT_FMT_TXT_CMAP_SPARSE_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

/*Store all the custom data of the font*/
static lv_font_fmt_txt_dsc_t font_dsc = {
    .glyph_bitmap = gylph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 2,
    .bpp = 2,
    .kern_classes = 0,
    .bitmap_format = 1
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
lv_font_t Sceta_20 = {
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 24,          /*The maximum line height required by the font*/
    .base_line = 0,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_HOR,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};

#endif /*#if SCETA_20*/
