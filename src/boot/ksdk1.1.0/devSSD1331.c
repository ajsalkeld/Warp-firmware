#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];

GFXfont * devSSD1331font = &FreeSans8pt7b;

void devSSD1331SetupSPI (void)
{
    /*
     *	Override Warp firmware's use of these pins.
     *
     *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

    warpEnableSPIpins();

    /*
     *	Override Warp firmware's use of these pins.
     *
     *	Reconfigure to use as GPIO.
     */
    PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
    PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);
}

static int writeCommand(uint8_t commandByte)
{
    spi_status_t status;
    if (!spi_enabled) devSSD1331SetupSPI();


    /*
     *	Drive /CS low.
     *
     *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
     */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}



int devSSD1331init(void)
{
    if (!spi_enabled) devSSD1331SetupSPI();


    /*
     *	RST high->low->high.
     */
    GPIO_DRV_SetPinOutput(kSSD1331PinRST);
    OSA_TimeDelay(100);
    GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
    OSA_TimeDelay(100);
    GPIO_DRV_SetPinOutput(kSSD1331PinRST);
    OSA_TimeDelay(100);

    /*
     *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
     */
    writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
    writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
    writeCommand(0x72);				// RGB Color
    writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
    writeCommand(0x0);
    writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
    writeCommand(0x0);
    writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
    writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
    writeCommand(0x3F);				// 0x3F 1/64 duty
    writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
    writeCommand(0x8E);
    writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
    writeCommand(0x0B);
    writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
    writeCommand(0x31);
    writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
    writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
    writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
    writeCommand(0x64);
    writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
    writeCommand(0x78);
    writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
    writeCommand(0x64);
    writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
    writeCommand(0x3A);
    writeCommand(kSSD1331CommandVCOMH);		// 0xBE
    writeCommand(0x3E);
    writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
    writeCommand(0x0F); // Give it all it's got!
    writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
    writeCommand(0x91);
    writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
    writeCommand(0xFF); // Max contrast Green
    writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
    writeCommand(0x7D);
    writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

    /*
     *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
     */
    writeCommand(kSSD1331CommandFILL);
    writeCommand(0x01);

    /*
     *	Clear Screen
     */
    writeCommand(kSSD1331CommandCLEAR);
    writeCommand(0x00);
    writeCommand(0x00);
    writeCommand(0x5F);
    writeCommand(0x3F);



    /*
     *	Any post-initialization drawing commands go here.
     */
    struct RGB light_green = {
            0x10, 0x3F, 0x00
    };
    struct RGB black = {
            0x00, 0x00, 0x00
    };

    int status = devSSD1331DrawRectangle(0x00, 0x3E, 0x00, 0x5E, light_green, light_green);

    //char test_str[] = "Hello World!";
    //devSSD1331print(10,10,test_str,black);

    if (status != kWarpStatusOK)     SEGGER_RTT_WriteString(0, "\r\n\tStatus not OK \n");

    SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle... \n");

    return 0;
}

int devSSD1331DrawRectangle (uint8_t first_row, uint8_t last_row, uint8_t first_col, uint8_t last_col,
                             RGB line, RGB fill)
{
    writeCommand(kSSD1331CommandDRAWRECT);
    writeCommand(first_col);     // First Col
    writeCommand(first_row);     // First Row
    writeCommand(last_col);     // Last Col = 94 = 0x5E
    writeCommand(last_row);     // Last Row = 62 = 0x3E
    writeCommand(line.R);     // Line R
    writeCommand(line.G);     // Line G
    writeCommand(line.B);     // Line B
    writeCommand(fill.R);     // Fill R
    writeCommand(fill.G);     // Fill G
    int status = writeCommand(fill.B);    // Fill B

    return status;
}

int devSSD1331DrawLine (uint8_t first_row, uint8_t last_row, uint8_t first_col, uint8_t last_col, RGB line)
{
    writeCommand(kSSD1331CommandDRAWLINE);
    writeCommand(first_col);     // First Col
    writeCommand(first_row);     // First Row
    writeCommand(last_col);     // Last Col = 94 = 0x5E
    writeCommand(last_row);     // Last Row = 62 = 0x3E
    writeCommand(line.R);     // Line R
    writeCommand(line.G);     // Line G
    int status = writeCommand(line.B);     // Line B

    return status;
}

int devSSD1331DrawChar (uint8_t x, uint8_t y, unsigned char c, RGB color, uint8_t size_x, uint8_t size_y)
{
    int status;

    // Ported from https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Adafruit_GFX.cpp#L1134
    c -= (uint8_t) devSSD1331font->first;
    GFXglyph *glyph = devSSD1331font->glyph + c;
    uint16_t    bo  = glyph->bitmapOffset;
    uint8_t     w   = glyph->width;
    uint8_t     h   = glyph->height;
    int8_t      xo  = glyph->xOffset;
    int8_t      yo  = glyph->yOffset;
    uint8_t     xx, yy;
    uint8_t     bits = 0, bit = 0;
    int16_t     xo16 = 0, yo16 = 0;

    if (size_x > 1 || size_y > 1) {
        xo16 = xo;
        yo16 = yo;
    }

    for (yy = 0; yy < h; ++yy)
    {
        for (xx = 0; xx < w; ++xx)
        {
            if (!(bit++ & 7))
            {
                bits = devSSD1331font->bitmap[bo++];
            }
            if (bits & 0x80)
            {
                if (size_x == 1 && size_y == 1)
                {
                    status = devSSD1331DrawPixel(x + xo + xx, y + yo + yy, color);
                }
                else
                {
                    status = devSSD1331DrawRectangle(y + (yo16 + yy) * size_y, y + (yo16 + yy) * size_y + size_y,
                                                     x + (xo16 + xx) * size_x, x + (xo16 + xx) * size_x + size_x,
                                                     color, color);
                }
            }
            bits <<= 1;
        }
    }

    return status;
}

int devSSD1331print (uint8_t x, uint8_t y, char* c, RGB color)
{
    int status = 0;

    for (int i = 0; c[i] != '\0'; ++i)
    {
        // Wrap text
        if (x > (SSD1331_WIDTH - (devSSD1331font->glyph + c[i] - devSSD1331font->first)->width ))
        {
            x = 0;
            y += devSSD1331font->yAdvance;
        }

        // Check scree-space
        if (y > (SSD1331_HEIGHT - devSSD1331font->yAdvance))
        {
            return 1;
        }

        status = devSSD1331DrawChar(x, y, c[i], color, 1, 1);

        x += (devSSD1331font->glyph + c[i] - devSSD1331font->first)->xAdvance;
    }

    return status;
}
