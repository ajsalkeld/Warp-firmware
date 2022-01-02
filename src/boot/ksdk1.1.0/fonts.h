//
// Created by andrew on 02/01/2022.
//

#ifndef WARP_FIRMWARE_FONTS_H
#define WARP_FIRMWARE_FONTS_H

// From Adafruit's GFXfont https://github.com/adafruit/Adafruit-GFX-Library/blob/master/gfxfont.h
/// Font data stored PER GLYPH
typedef struct {
    uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
    uint8_t width;         ///< Bitmap dimensions in pixels
    uint8_t height;        ///< Bitmap dimensions in pixels
    uint8_t xAdvance;      ///< Distance to advance cursor (x axis)
    int8_t xOffset;        ///< X dist from cursor pos to UL corner
    int8_t yOffset;        ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct {
    uint8_t *bitmap;  ///< Glyph bitmaps, concatenated
    GFXglyph *glyph;  ///< Glyph array
    uint16_t first;   ///< ASCII extents (first char)
    uint16_t last;    ///< ASCII extents (last char)
    uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;

// Generated by Adafruit's fontconvert https://github.com/adafruit/Adafruit-GFX-Library/tree/master/fontconvert
const unsigned char FreeSans8pt7bBitmaps[] = {
        0x00, 0xFF, 0xB0, 0xBB, 0xA8, 0x13, 0x09, 0x04, 0x8F, 0xF3, 0x21, 0x90,
        0x99, 0xFE, 0x24, 0x12, 0x19, 0x00, 0x10, 0xFA, 0x5C, 0x99, 0x1E, 0x0F,
        0x0B, 0x13, 0x26, 0x5B, 0xE1, 0x02, 0x00, 0x38, 0x41, 0x31, 0x0C, 0x48,
        0x11, 0x20, 0x79, 0x00, 0x04, 0x00, 0x27, 0x81, 0xB3, 0x04, 0x84, 0x23,
        0x30, 0x87, 0x80, 0x3C, 0x32, 0x11, 0x05, 0x83, 0x83, 0xC3, 0x35, 0x0E,
        0x86, 0x63, 0x9F, 0x60, 0xF0, 0x12, 0x64, 0x4C, 0xC8, 0x8C, 0xC4, 0x42,
        0x20, 0x91, 0x26, 0x49, 0x24, 0xA4, 0xA0, 0x22, 0xF6, 0x90, 0x10, 0x10,
        0x10, 0x10, 0xFF, 0x10, 0x10, 0x10, 0xF5, 0xF0, 0xF0, 0x00, 0x84, 0x22,
        0x10, 0x88, 0x42, 0x21, 0x00, 0x7D, 0x8B, 0x1C, 0x18, 0x30, 0x60, 0xC1,
        0x87, 0x89, 0xF0, 0x37, 0xF3, 0x33, 0x33, 0x33, 0x30, 0x7D, 0x8E, 0x0C,
        0x10, 0x61, 0x8E, 0x30, 0xC1, 0x03, 0xF8, 0x7D, 0x8E, 0x18, 0x30, 0xC3,
        0x81, 0x81, 0x83, 0x8D, 0xF0, 0x06, 0x0E, 0x0E, 0x16, 0x26, 0x66, 0x46,
        0xFF, 0x06, 0x06, 0x06, 0x7E, 0x83, 0x06, 0x0F, 0xD8, 0xC0, 0x81, 0x83,
        0x8D, 0xF0, 0x7C, 0x8F, 0x0C, 0x0B, 0xD8, 0xE0, 0xC1, 0x83, 0x8D, 0xF0,
        0xFE, 0x04, 0x10, 0x60, 0x82, 0x04, 0x18, 0x20, 0x41, 0x80, 0x7D, 0x8E,
        0x1E, 0x37, 0xCF, 0xA1, 0xC1, 0x83, 0x8D, 0xF0, 0x7D, 0x8A, 0x1C, 0x18,
        0x78, 0xDE, 0x81, 0x87, 0x89, 0xE0, 0xC0, 0x0F, 0xF0, 0x03, 0xD4, 0x01,
        0x07, 0x1C, 0xE0, 0xC0, 0x70, 0x0E, 0x03, 0xFF, 0x00, 0x00, 0xFF, 0x00,
        0xC0, 0x38, 0x0E, 0x03, 0x1C, 0x70, 0x80, 0x38, 0xDF, 0x0E, 0x10, 0x61,
        0x86, 0x08, 0x10, 0x00, 0x40, 0x80, 0x07, 0xC0, 0x73, 0xC3, 0x01, 0x98,
        0x03, 0xC7, 0xEE, 0x31, 0x18, 0x84, 0x66, 0x11, 0x98, 0xCE, 0x26, 0x6C,
        0xEF, 0x18, 0x00, 0x30, 0x00, 0x7F, 0x00, 0x0C, 0x03, 0x80, 0xA0, 0x68,
        0x13, 0x04, 0x43, 0x10, 0xFE, 0x7F, 0x98, 0x34, 0x0F, 0x01, 0xFC, 0x7F,
        0xB0, 0x58, 0x2C, 0x17, 0xF3, 0xFD, 0x83, 0xC1, 0xE0, 0xF0, 0xFF, 0xE0,
        0x1E, 0x1D, 0xC4, 0x1B, 0x02, 0x80, 0x20, 0x08, 0x02, 0x00, 0x80, 0xF0,
        0x26, 0x18, 0xFC, 0xFC, 0x3F, 0xCC, 0x1B, 0x02, 0xC0, 0xB0, 0x3C, 0x0F,
        0x02, 0xC0, 0xB0, 0x6C, 0x33, 0xF8, 0xFF, 0x7F, 0xB0, 0x18, 0x0C, 0x07,
        0xFB, 0xFD, 0x80, 0xC0, 0x60, 0x30, 0x1F, 0xF0, 0xFF, 0xFF, 0xC0, 0xC0,
        0xC0, 0xFE, 0xFE, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x1E, 0x1E, 0xE4, 0x0F,
        0x03, 0x80, 0x20, 0x08, 0x7E, 0x01, 0xC0, 0x70, 0x36, 0x1C, 0xFD, 0xC0,
        0xE0, 0x70, 0x38, 0x1C, 0x0F, 0xFF, 0xFF, 0x81, 0xC0, 0xE0, 0x70, 0x38,
        0x10, 0xFF, 0xF0, 0x04, 0x10, 0x41, 0x04, 0x10, 0x41, 0x86, 0x18, 0xFE,
        0xC1, 0xB0, 0xCC, 0x63, 0x30, 0xD8, 0x3E, 0x0E, 0xC3, 0x10, 0xC6, 0x30,
        0xCC, 0x1B, 0x06, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
        0xC0, 0xC0, 0xFF, 0xC0, 0x7C, 0x0F, 0x83, 0xF0, 0x5F, 0x0B, 0xA3, 0x74,
        0x4E, 0xC9, 0xCB, 0x39, 0x47, 0x38, 0xE3, 0x10, 0xC0, 0xF0, 0x78, 0x3E,
        0x1D, 0x8E, 0x47, 0x33, 0x89, 0xC6, 0xE1, 0xF0, 0x78, 0x30, 0x1E, 0x07,
        0x71, 0x83, 0x60, 0x28, 0x07, 0x00, 0xE0, 0x1C, 0x03, 0xC0, 0x78, 0x19,
        0x87, 0x1F, 0x80, 0xFC, 0x7F, 0xB0, 0x78, 0x3C, 0x1E, 0x1B, 0xF9, 0x80,
        0xC0, 0x60, 0x30, 0x18, 0x00, 0x1E, 0x07, 0x71, 0x83, 0x60, 0x28, 0x07,
        0x00, 0xE0, 0x1C, 0x03, 0xC0, 0x78, 0x79, 0x86, 0x1F, 0xE0, 0x04, 0xFE,
        0x3F, 0xEC, 0x1B, 0x02, 0xC0, 0xB0, 0x4F, 0xF3, 0x06, 0xC1, 0xB0, 0x2C,
        0x0B, 0x02, 0x3C, 0x33, 0xB0, 0x58, 0x3C, 0x03, 0xC0, 0x7C, 0x07, 0x81,
        0xC0, 0xF0, 0x4F, 0xE0, 0xFF, 0xFF, 0xF0, 0xC0, 0x30, 0x0C, 0x03, 0x00,
        0xC0, 0x30, 0x0C, 0x03, 0x00, 0xC0, 0x30, 0xC0, 0xE0, 0x70, 0x38, 0x1C,
        0x0E, 0x07, 0x03, 0x81, 0xC0, 0xE0, 0x58, 0x67, 0xE0, 0xC0, 0xD0, 0x36,
        0x09, 0x86, 0x21, 0x8C, 0x43, 0x30, 0x4C, 0x1A, 0x03, 0x80, 0xE0, 0x30,
        0xC1, 0x06, 0x87, 0x09, 0x0E, 0x13, 0x14, 0x66, 0x28, 0xC4, 0xD9, 0x09,
        0x12, 0x1A, 0x2C, 0x3C, 0x78, 0x38, 0xE0, 0x60, 0xC0, 0xC1, 0x80, 0x40,
        0xD8, 0x23, 0x18, 0x4C, 0x1A, 0x03, 0x00, 0xC0, 0x78, 0x13, 0x0C, 0x66,
        0x19, 0x03, 0xC0, 0x6C, 0x18, 0x82, 0x18, 0xC1, 0xB0, 0x14, 0x03, 0x80,
        0x20, 0x04, 0x00, 0x80, 0x10, 0x02, 0x00, 0x7F, 0xBF, 0xC0, 0x60, 0x60,
        0x60, 0x60, 0x30, 0x30, 0x30, 0x30, 0x18, 0x1F, 0xF0, 0xF2, 0x49, 0x24,
        0x92, 0x49, 0x38, 0x84, 0x20, 0x84, 0x20, 0x84, 0x20, 0x84, 0x20, 0xE4,
        0x92, 0x49, 0x24, 0x92, 0x78, 0x31, 0xC5, 0x12, 0x8A, 0x10, 0xFF, 0x80,
        0x42, 0x10, 0x18, 0x7E, 0x42, 0x02, 0x3E, 0x62, 0x42, 0x46, 0x7F, 0x81,
        0x02, 0x04, 0xCF, 0xD8, 0xE0, 0xC1, 0x83, 0x87, 0x1D, 0xE0, 0x18, 0x3E,
        0x42, 0x40, 0xC0, 0x40, 0x43, 0x62, 0x3E, 0x01, 0x01, 0x01, 0x19, 0x3F,
        0x63, 0x43, 0xC1, 0x41, 0x43, 0x63, 0x3D, 0x18, 0x3E, 0x43, 0x41, 0xFF,
        0x40, 0x41, 0x63, 0x3E, 0x32, 0x66, 0xF6, 0x66, 0x66, 0x66, 0x18, 0x3D,
        0x43, 0x43, 0xC3, 0x43, 0x43, 0x63, 0x3F, 0x03, 0x42, 0x7E, 0x81, 0x02,
        0x04, 0xCF, 0xD8, 0xE1, 0xC3, 0x87, 0x0E, 0x1C, 0x30, 0xCF, 0xF0, 0x6C,
        0x06, 0xDB, 0x6D, 0xB6, 0xF8, 0x81, 0x02, 0x04, 0x08, 0xD3, 0x2C, 0x78,
        0xD9, 0x1A, 0x34, 0x30, 0xFF, 0xF0, 0x18, 0xDF, 0xFF, 0x18, 0xC2, 0x18,
        0x43, 0x08, 0x61, 0x0C, 0x21, 0x84, 0x20, 0x19, 0x7B, 0x1C, 0x38, 0x70,
        0xE1, 0xC3, 0x86, 0x18, 0x3E, 0x63, 0x41, 0xC1, 0x41, 0x43, 0x63, 0x3E,
        0x19, 0xFB, 0x1C, 0x18, 0x30, 0x70, 0xE3, 0xBD, 0x02, 0x04, 0x00, 0x18,
        0x3F, 0x63, 0x43, 0xC1, 0x41, 0x43, 0x63, 0x3D, 0x01, 0x01, 0x01, 0x1B,
        0xC8, 0x88, 0x88, 0x80, 0x33, 0xF8, 0x60, 0xF0, 0x78, 0x61, 0xFC, 0x66,
        0x6F, 0x66, 0x66, 0x66, 0x30, 0x87, 0x0E, 0x1C, 0x38, 0x70, 0xF1, 0xFF,
        0xC2, 0x42, 0x66, 0x24, 0x24, 0x3C, 0x18, 0x18, 0xC6, 0x29, 0xCD, 0x29,
        0xB5, 0xA2, 0x94, 0x73, 0x8C, 0x61, 0x8C, 0x46, 0x48, 0xE0, 0xC1, 0x85,
        0x99, 0x21, 0xC2, 0x8D, 0x93, 0x22, 0xC7, 0x0E, 0x0C, 0x10, 0x61, 0x80,
        0x7E, 0x08, 0x30, 0xC3, 0x04, 0x10, 0x7F, 0x2D, 0x24, 0x94, 0xC9, 0x24,
        0x98, 0x7F, 0xFE, 0x84, 0x44, 0x44, 0x63, 0x26, 0x44, 0x44, 0xC0, 0xE2,
        0xD0, 0xC0 };

const GFXglyph FreeSans8pt7bGlyphs[] = {
        {     0,   1,   1,   4,    0,    0 },   // 0x20 ' '
        {     1,   1,  12,   5,    2,  -11 },   // 0x21 '!'
        {     3,   4,   4,   5,    1,  -10 },   // 0x22 '"'
        {     5,   9,  11,   9,    0,  -10 },   // 0x23 '#'
        {    18,   7,  14,   9,    1,  -11 },   // 0x24 '$'
        {    31,  14,  11,  14,    0,  -10 },   // 0x25 '%'
        {    51,   9,  11,  11,    1,  -10 },   // 0x26 '&'
        {    64,   1,   4,   3,    1,  -10 },   // 0x27 '''
        {    65,   4,  15,   5,    1,  -11 },   // 0x28 '('
        {    73,   3,  15,   5,    1,  -11 },   // 0x29 ')'
        {    79,   4,   5,   6,    1,  -11 },   // 0x2A '*'
        {    82,   8,   8,   9,    1,   -7 },   // 0x2B '+'
        {    90,   2,   4,   4,    1,   -1 },   // 0x2C ','
        {    91,   4,   1,   5,    1,   -4 },   // 0x2D '-'
        {    92,   2,   2,   4,    1,   -1 },   // 0x2E '.'
        {    93,   5,  12,   4,    0,  -11 },   // 0x2F '/'
        {   101,   7,  11,   9,    1,  -10 },   // 0x30 '0'
        {   111,   4,  11,   9,    2,  -10 },   // 0x31 '1'
        {   117,   7,  11,   9,    1,  -10 },   // 0x32 '2'
        {   127,   7,  11,   9,    1,  -10 },   // 0x33 '3'
        {   137,   8,  11,   9,    0,  -10 },   // 0x34 '4'
        {   148,   7,  11,   9,    1,  -10 },   // 0x35 '5'
        {   158,   7,  11,   9,    1,  -10 },   // 0x36 '6'
        {   168,   7,  11,   9,    1,  -10 },   // 0x37 '7'
        {   178,   7,  11,   9,    1,  -10 },   // 0x38 '8'
        {   188,   7,  11,   9,    1,  -10 },   // 0x39 '9'
        {   198,   2,   8,   4,    1,   -7 },   // 0x3A ':'
        {   200,   2,  11,   4,    1,   -8 },   // 0x3B ';'
        {   203,   8,   8,   9,    1,   -7 },   // 0x3C '<'
        {   211,   8,   4,   9,    1,   -5 },   // 0x3D '='
        {   215,   8,   8,   9,    1,   -7 },   // 0x3E '>'
        {   223,   7,  12,   9,    1,  -11 },   // 0x3F '?'
        {   234,  14,  14,  16,    1,  -11 },   // 0x40 '@'
        {   259,  10,  12,  11,    0,  -11 },   // 0x41 'A'
        {   274,   9,  12,  11,    1,  -11 },   // 0x42 'B'
        {   288,  10,  12,  11,    1,  -11 },   // 0x43 'C'
        {   303,  10,  12,  11,    1,  -11 },   // 0x44 'D'
        {   318,   9,  12,  10,    1,  -11 },   // 0x45 'E'
        {   332,   8,  12,  10,    1,  -11 },   // 0x46 'F'
        {   344,  10,  12,  12,    1,  -11 },   // 0x47 'G'
        {   359,   9,  12,  12,    1,  -11 },   // 0x48 'H'
        {   373,   1,  12,   4,    2,  -11 },   // 0x49 'I'
        {   375,   6,  12,   8,    1,  -11 },   // 0x4A 'J'
        {   384,  10,  12,  11,    1,  -11 },   // 0x4B 'K'
        {   399,   8,  12,   9,    1,  -11 },   // 0x4C 'L'
        {   411,  11,  12,  14,    1,  -11 },   // 0x4D 'M'
        {   428,   9,  12,  12,    1,  -11 },   // 0x4E 'N'
        {   442,  11,  12,  13,    1,  -11 },   // 0x4F 'O'
        {   459,   9,  12,  11,    1,  -11 },   // 0x50 'P'
        {   473,  11,  13,  13,    1,  -11 },   // 0x51 'Q'
        {   491,  10,  12,  11,    1,  -11 },   // 0x52 'R'
        {   506,   9,  12,  11,    1,  -11 },   // 0x53 'S'
        {   520,  10,  12,  10,    0,  -11 },   // 0x54 'T'
        {   535,   9,  12,  12,    1,  -11 },   // 0x55 'U'
        {   549,  10,  12,  10,    0,  -11 },   // 0x56 'V'
        {   564,  15,  12,  15,    0,  -11 },   // 0x57 'W'
        {   587,  10,  12,  11,    0,  -11 },   // 0x58 'X'
        {   602,  11,  12,  11,    0,  -11 },   // 0x59 'Y'
        {   619,   9,  12,  10,    0,  -11 },   // 0x5A 'Z'
        {   633,   3,  15,   4,    1,  -11 },   // 0x5B '['
        {   639,   5,  12,   4,    0,  -11 },   // 0x5C '\'
        {   647,   3,  15,   4,    0,  -11 },   // 0x5D ']'
        {   653,   6,   6,   8,    1,  -10 },   // 0x5E '^'
        {   658,   9,   1,   9,    0,    3 },   // 0x5F '_'
        {   660,   4,   3,   4,    0,  -11 },   // 0x60 '`'
        {   662,   8,   9,   9,    0,   -8 },   // 0x61 'a'
        {   671,   7,  12,   9,    1,  -11 },   // 0x62 'b'
        {   682,   8,   9,   8,    0,   -8 },   // 0x63 'c'
        {   691,   8,  12,   9,    0,  -11 },   // 0x64 'd'
        {   703,   8,   9,   9,    0,   -8 },   // 0x65 'e'
        {   712,   4,  12,   4,    0,  -11 },   // 0x66 'f'
        {   718,   8,  12,   9,    0,   -8 },   // 0x67 'g'
        {   730,   7,  12,   9,    1,  -11 },   // 0x68 'h'
        {   741,   1,  12,   4,    1,  -11 },   // 0x69 'i'
        {   743,   3,  15,   4,    0,  -11 },   // 0x6A 'j'
        {   749,   7,  12,   8,    1,  -11 },   // 0x6B 'k'
        {   760,   1,  12,   3,    1,  -11 },   // 0x6C 'l'
        {   762,  11,   9,  13,    1,   -8 },   // 0x6D 'm'
        {   775,   7,   9,   9,    1,   -8 },   // 0x6E 'n'
        {   783,   8,   9,   9,    0,   -8 },   // 0x6F 'o'
        {   792,   7,  12,   9,    1,   -8 },   // 0x70 'p'
        {   803,   8,  12,   9,    0,   -8 },   // 0x71 'q'
        {   815,   4,   9,   5,    1,   -8 },   // 0x72 'r'
        {   820,   6,   9,   8,    1,   -8 },   // 0x73 's'
        {   827,   4,  11,   4,    0,  -10 },   // 0x74 't'
        {   833,   7,   8,   9,    1,   -7 },   // 0x75 'u'
        {   840,   8,   8,   8,    0,   -7 },   // 0x76 'v'
        {   848,  11,   8,  12,    0,   -7 },   // 0x77 'w'
        {   859,   7,   8,   8,    0,   -7 },   // 0x78 'x'
        {   866,   7,  11,   8,    0,   -7 },   // 0x79 'y'
        {   876,   7,   8,   8,    0,   -7 },   // 0x7A 'z'
        {   883,   3,  15,   5,    1,  -11 },   // 0x7B '{'
        {   889,   1,  15,   4,    2,  -11 },   // 0x7C '|'
        {   891,   4,  15,   5,    1,  -11 },   // 0x7D '}'
        {   899,   6,   3,   8,    1,   -6 } }; // 0x7E '~'

const GFXfont FreeSans8pt7b = {
        FreeSans8pt7bBitmaps,
        FreeSans8pt7bGlyphs,
        0x20, 0x7E, 17
};

// Approx. 1574 bytes

// Org_v01 by Orgdot (www.orgdot.com/aliasfonts).  A tiny,
// stylized font with all characters within a 6 pixel height.

const uint8_t Org_01Bitmaps[] = {
        0xE8, 0xA0, 0x57, 0xD5, 0xF5, 0x00, 0xFD, 0x3E, 0x5F, 0x80, 0x88, 0x88,
        0x88, 0x80, 0xF4, 0xBF, 0x2E, 0x80, 0x80, 0x6A, 0x40, 0x95, 0x80, 0xAA,
        0x80, 0x5D, 0x00, 0xC0, 0xF0, 0x80, 0x08, 0x88, 0x88, 0x00, 0xFC, 0x63,
        0x1F, 0x80, 0xF8, 0xF8, 0x7F, 0x0F, 0x80, 0xF8, 0x7E, 0x1F, 0x80, 0x8C,
        0x7E, 0x10, 0x80, 0xFC, 0x3E, 0x1F, 0x80, 0xFC, 0x3F, 0x1F, 0x80, 0xF8,
        0x42, 0x10, 0x80, 0xFC, 0x7F, 0x1F, 0x80, 0xFC, 0x7E, 0x1F, 0x80, 0x90,
        0xB0, 0x2A, 0x22, 0xF0, 0xF0, 0x88, 0xA8, 0xF8, 0x4E, 0x02, 0x00, 0xFD,
        0x6F, 0x0F, 0x80, 0xFC, 0x7F, 0x18, 0x80, 0xF4, 0x7D, 0x1F, 0x00, 0xFC,
        0x21, 0x0F, 0x80, 0xF4, 0x63, 0x1F, 0x00, 0xFC, 0x3F, 0x0F, 0x80, 0xFC,
        0x3F, 0x08, 0x00, 0xFC, 0x2F, 0x1F, 0x80, 0x8C, 0x7F, 0x18, 0x80, 0xF9,
        0x08, 0x4F, 0x80, 0x78, 0x85, 0x2F, 0x80, 0x8D, 0xB1, 0x68, 0x80, 0x84,
        0x21, 0x0F, 0x80, 0xFD, 0x6B, 0x5A, 0x80, 0xFC, 0x63, 0x18, 0x80, 0xFC,
        0x63, 0x1F, 0x80, 0xFC, 0x7F, 0x08, 0x00, 0xFC, 0x63, 0x3F, 0x80, 0xFC,
        0x7F, 0x29, 0x00, 0xFC, 0x3E, 0x1F, 0x80, 0xF9, 0x08, 0x42, 0x00, 0x8C,
        0x63, 0x1F, 0x80, 0x8C, 0x62, 0xA2, 0x00, 0xAD, 0x6B, 0x5F, 0x80, 0x8A,
        0x88, 0xA8, 0x80, 0x8C, 0x54, 0x42, 0x00, 0xF8, 0x7F, 0x0F, 0x80, 0xEA,
        0xC0, 0x82, 0x08, 0x20, 0x80, 0xD5, 0xC0, 0x54, 0xF8, 0x80, 0xF1, 0xFF,
        0x8F, 0x99, 0xF0, 0xF8, 0x8F, 0x1F, 0x99, 0xF0, 0xFF, 0x8F, 0x6B, 0xA4,
        0xF9, 0x9F, 0x10, 0x8F, 0x99, 0x90, 0xF0, 0x55, 0xC0, 0x8A, 0xF9, 0x90,
        0xF8, 0xFD, 0x63, 0x10, 0xF9, 0x99, 0xF9, 0x9F, 0xF9, 0x9F, 0x80, 0xF9,
        0x9F, 0x20, 0xF8, 0x88, 0x47, 0x1F, 0x27, 0xC8, 0x42, 0x00, 0x99, 0x9F,
        0x99, 0x97, 0x8C, 0x6B, 0xF0, 0x96, 0x69, 0x99, 0x9F, 0x10, 0x2E, 0x8F,
        0x2B, 0x22, 0xF8, 0x89, 0xA8, 0x0F, 0xE0};

const GFXglyph Org_01Glyphs[] = {{0, 0, 0, 6, 0, 1},     // 0x20 ' '
                                         {0, 1, 5, 2, 0, -4},    // 0x21 '!'
                                         {1, 3, 1, 4, 0, -4},    // 0x22 '"'
                                         {2, 5, 5, 6, 0, -4},    // 0x23 '#'
                                         {6, 5, 5, 6, 0, -4},    // 0x24 '$'
                                         {10, 5, 5, 6, 0, -4},   // 0x25 '%'
                                         {14, 5, 5, 6, 0, -4},   // 0x26 '&'
                                         {18, 1, 1, 2, 0, -4},   // 0x27 '''
                                         {19, 2, 5, 3, 0, -4},   // 0x28 '('
                                         {21, 2, 5, 3, 0, -4},   // 0x29 ')'
                                         {23, 3, 3, 4, 0, -3},   // 0x2A '*'
                                         {25, 3, 3, 4, 0, -3},   // 0x2B '+'
                                         {27, 1, 2, 2, 0, 0},    // 0x2C ','
                                         {28, 4, 1, 5, 0, -2},   // 0x2D '-'
                                         {29, 1, 1, 2, 0, 0},    // 0x2E '.'
                                         {30, 5, 5, 6, 0, -4},   // 0x2F '/'
                                         {34, 5, 5, 6, 0, -4},   // 0x30 '0'
                                         {38, 1, 5, 2, 0, -4},   // 0x31 '1'
                                         {39, 5, 5, 6, 0, -4},   // 0x32 '2'
                                         {43, 5, 5, 6, 0, -4},   // 0x33 '3'
                                         {47, 5, 5, 6, 0, -4},   // 0x34 '4'
                                         {51, 5, 5, 6, 0, -4},   // 0x35 '5'
                                         {55, 5, 5, 6, 0, -4},   // 0x36 '6'
                                         {59, 5, 5, 6, 0, -4},   // 0x37 '7'
                                         {63, 5, 5, 6, 0, -4},   // 0x38 '8'
                                         {67, 5, 5, 6, 0, -4},   // 0x39 '9'
                                         {71, 1, 4, 2, 0, -3},   // 0x3A ':'
                                         {72, 1, 4, 2, 0, -3},   // 0x3B ';'
                                         {73, 3, 5, 4, 0, -4},   // 0x3C '<'
                                         {75, 4, 3, 5, 0, -3},   // 0x3D '='
                                         {77, 3, 5, 4, 0, -4},   // 0x3E '>'
                                         {79, 5, 5, 6, 0, -4},   // 0x3F '?'
                                         {83, 5, 5, 6, 0, -4},   // 0x40 '@'
                                         {87, 5, 5, 6, 0, -4},   // 0x41 'A'
                                         {91, 5, 5, 6, 0, -4},   // 0x42 'B'
                                         {95, 5, 5, 6, 0, -4},   // 0x43 'C'
                                         {99, 5, 5, 6, 0, -4},   // 0x44 'D'
                                         {103, 5, 5, 6, 0, -4},  // 0x45 'E'
                                         {107, 5, 5, 6, 0, -4},  // 0x46 'F'
                                         {111, 5, 5, 6, 0, -4},  // 0x47 'G'
                                         {115, 5, 5, 6, 0, -4},  // 0x48 'H'
                                         {119, 5, 5, 6, 0, -4},  // 0x49 'I'
                                         {123, 5, 5, 6, 0, -4},  // 0x4A 'J'
                                         {127, 5, 5, 6, 0, -4},  // 0x4B 'K'
                                         {131, 5, 5, 6, 0, -4},  // 0x4C 'L'
                                         {135, 5, 5, 6, 0, -4},  // 0x4D 'M'
                                         {139, 5, 5, 6, 0, -4},  // 0x4E 'N'
                                         {143, 5, 5, 6, 0, -4},  // 0x4F 'O'
                                         {147, 5, 5, 6, 0, -4},  // 0x50 'P'
                                         {151, 5, 5, 6, 0, -4},  // 0x51 'Q'
                                         {155, 5, 5, 6, 0, -4},  // 0x52 'R'
                                         {159, 5, 5, 6, 0, -4},  // 0x53 'S'
                                         {163, 5, 5, 6, 0, -4},  // 0x54 'T'
                                         {167, 5, 5, 6, 0, -4},  // 0x55 'U'
                                         {171, 5, 5, 6, 0, -4},  // 0x56 'V'
                                         {175, 5, 5, 6, 0, -4},  // 0x57 'W'
                                         {179, 5, 5, 6, 0, -4},  // 0x58 'X'
                                         {183, 5, 5, 6, 0, -4},  // 0x59 'Y'
                                         {187, 5, 5, 6, 0, -4},  // 0x5A 'Z'
                                         {191, 2, 5, 3, 0, -4},  // 0x5B '['
                                         {193, 5, 5, 6, 0, -4},  // 0x5C '\'
                                         {197, 2, 5, 3, 0, -4},  // 0x5D ']'
                                         {199, 3, 2, 4, 0, -4},  // 0x5E '^'
                                         {200, 5, 1, 6, 0, 1},   // 0x5F '_'
                                         {201, 1, 1, 2, 0, -4},  // 0x60 '`'
                                         {202, 4, 4, 5, 0, -3},  // 0x61 'a'
                                         {204, 4, 5, 5, 0, -4},  // 0x62 'b'
                                         {207, 4, 4, 5, 0, -3},  // 0x63 'c'
                                         {209, 4, 5, 5, 0, -4},  // 0x64 'd'
                                         {212, 4, 4, 5, 0, -3},  // 0x65 'e'
                                         {214, 3, 5, 4, 0, -4},  // 0x66 'f'
                                         {216, 4, 5, 5, 0, -3},  // 0x67 'g'
                                         {219, 4, 5, 5, 0, -4},  // 0x68 'h'
                                         {222, 1, 4, 2, 0, -3},  // 0x69 'i'
                                         {223, 2, 5, 3, 0, -3},  // 0x6A 'j'
                                         {225, 4, 5, 5, 0, -4},  // 0x6B 'k'
                                         {228, 1, 5, 2, 0, -4},  // 0x6C 'l'
                                         {229, 5, 4, 6, 0, -3},  // 0x6D 'm'
                                         {232, 4, 4, 5, 0, -3},  // 0x6E 'n'
                                         {234, 4, 4, 5, 0, -3},  // 0x6F 'o'
                                         {236, 4, 5, 5, 0, -3},  // 0x70 'p'
                                         {239, 4, 5, 5, 0, -3},  // 0x71 'q'
                                         {242, 4, 4, 5, 0, -3},  // 0x72 'r'
                                         {244, 4, 4, 5, 0, -3},  // 0x73 's'
                                         {246, 5, 5, 6, 0, -4},  // 0x74 't'
                                         {250, 4, 4, 5, 0, -3},  // 0x75 'u'
                                         {252, 4, 4, 5, 0, -3},  // 0x76 'v'
                                         {254, 5, 4, 6, 0, -3},  // 0x77 'w'
                                         {257, 4, 4, 5, 0, -3},  // 0x78 'x'
                                         {259, 4, 5, 5, 0, -3},  // 0x79 'y'
                                         {262, 4, 4, 5, 0, -3},  // 0x7A 'z'
                                         {264, 3, 5, 4, 0, -4},  // 0x7B '{'
                                         {266, 1, 5, 2, 0, -4},  // 0x7C '|'
                                         {267, 3, 5, 4, 0, -4},  // 0x7D '}'
                                         {269, 5, 3, 6, 0, -3}}; // 0x7E '~'

const GFXfont Org_01 = {(uint8_t *)Org_01Bitmaps,
                                (GFXglyph *)Org_01Glyphs, 0x20, 0x7E, 7};

// Approx. 943 bytes

#endif //WARP_FIRMWARE_FONTS_H
