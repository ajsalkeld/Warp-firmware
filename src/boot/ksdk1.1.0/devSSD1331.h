/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 */
#ifndef DEVSSD1331_H
#define DEVSSD1331_H
#include <stdint.h>
#include "fonts.h"

#define SSD1331_HEIGHT 64
#define SSD1331_WIDTH 96

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;

typedef enum
{
	kSSD1331CommandDRAWLINE		= 0x21,
	kSSD1331CommandDRAWRECT		= 0x22,
	kSSD1331CommandCLEAR		= 0x25,
	kSSD1331CommandFILL		    = 0x26,
	kSSD1331CommandSETCOLUMN	= 0x15,
	kSSD1331CommandSETROW		= 0x75,
	kSSD1331CommandCONTRASTA	= 0x81,
	kSSD1331CommandCONTRASTB	= 0x82,
	kSSD1331CommandCONTRASTC	= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP		= 0xA0,
	kSSD1331CommandSTARTLINE	= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON	= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX	= 0xA8,
	kSSD1331CommandSETMASTER	= 0xAD,
	kSSD1331CommandDISPLAYOFF	= 0xAE,
	kSSD1331CommandDISPLAYON	= 0xAF,
	kSSD1331CommandPOWERMODE	= 0xB0,
	kSSD1331CommandPRECHARGE	= 0xB1,
	kSSD1331CommandCLOCKDIV		= 0xB3,
	kSSD1331CommandPRECHARGEA	= 0x8A,
	kSSD1331CommandPRECHARGEB	= 0x8B,
	kSSD1331CommandPRECHARGEC	= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH		= 0xBE,
} SSD1331Commands;

typedef struct {
    uint16_t R;
    uint16_t G;
    uint16_t B;
} RGB;

RGB light_green = {
        0x10, 0x3F, 0x00
};

RGB black = {
        0x00, 0x00, 0x00
};

RGB white = {
        0xFF, 0xFF, 0xFF
};

extern GFXfont * devSSD1331font;

int	devSSD1331init(void);

int devSSD1331DrawRectangle (uint8_t first_row, uint8_t last_row, uint8_t first_col, uint8_t last_col,
                             RGB line, RGB fill);

int devSSD1331DrawLine (uint8_t first_row, uint8_t last_row, uint8_t first_col, uint8_t last_col, RGB line);

#define devSSD1331DrawPixel(x, y, color) devSSD1331DrawLine(y,y,x,x,color)

void devSSD1331SetFont (GFXfont new_font) {devSSD1331font = &new_font;}

int devSSD1331DrawChar (uint8_t x, uint8_t y, unsigned char c, RGB color, uint8_t size_x, uint8_t size_y);

int devSSD1331print (uint8_t x, uint8_t y, char* c, RGB color);

void devSSD1331SetupSPI (void);

int devSSD1331ClearScreen (void);

static int devSSD1331WriteCommand (uint8_t commandByte);

#endif