/*
	Authored 2021: Andrew Salkeld.

    Warp Firmware 2016-2018 Phillip Stanley-Marbell.

    FatFs (C) ChaN, 2015, All Rights Reserved.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/

// TODO: BROKEN


/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/

#include "config.h"
#include "fsl_spi_hal.h"
#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"
#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSDSPI.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];

#define	INIT_PORT()	devSDSPISetupSPI()	/* Initialize MMC control port (CS/CLK/DI:output, DO:input) */
#define DLY_MS(n)	OSA_TimeDelay(n)	/* Delay n milliseconds */

#define	CS_H()		GPIO_DRV_SetPinOutput(kSDSPIPinCSn)	/* Set MMC CS "high" */
#define CS_L()		GPIO_DRV_ClearPinOutput(kSDSPIPinCSn)	/* Set MMC CS "low" */

static void devSDSPISetupSPI (void)
{
    /*
     *	Override Warp firmware's use of these pins.
     *
     *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
     *	MISO on PTA6 as default.
     */
    PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
    PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

    // SD CS
    PORT_HAL_SetMuxMode(PORTB_BASE, 1u, kPortMuxAsGpio);

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


/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/

/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define	ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

/* Card type flags (CardType) */
#define CT_MMC				0x01	/* MMC ver 3 */
#define CT_SD1				0x02	/* SD ver 1 */
#define CT_SD2				0x04	/* SD ver 2 */
#define CT_SDC				(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK			0x08	/* Block addressing */



static BYTE CardType;			/* b0:MMC, b1:SDv1, b2:SDv2, b3:Block addressing */



/*-----------------------------------------------------------------------*/
/* Transmit a byte to the MMC (bitbanging)                               */
/*-----------------------------------------------------------------------*/

inline static void xmit_mmc (BYTE d /* Data to be sent */)
{
    SPI_HAL_WriteData(g_spiBaseAddr[0], d);
}



/*-----------------------------------------------------------------------*/
/* Receive a byte from the MMC (bitbanging)                              */
/*-----------------------------------------------------------------------*/

inline static BYTE rcvr_mmc (void)
{
    return SPI_HAL_ReadData(g_spiBaseAddr[0]);
}

/*-----------------------------------------------------------------------*/
/* Skip bytes on the MMC (bitbanging)                                    */
/*-----------------------------------------------------------------------*/

inline static void skip_mmc (UINT n    /* Number of bytes to skip */)
{
    do {
        SPI_HAL_ReadData(g_spiBaseAddr[0]);
    } while (--n);
}



/*-----------------------------------------------------------------------*/
/* Deselect the card and release SPI bus                                 */
/*-----------------------------------------------------------------------*/

static
void release_spi (void)
{
    CS_H();
    rcvr_mmc();
    warpDisableSPIpins();
}


/*-----------------------------------------------------------------------*/
/* Send a command packet to MMC                                          */
/*-----------------------------------------------------------------------*/

static BYTE send_cmd (
        BYTE cmd,		/* Command byte */
        DWORD arg		/* Argument */
)
{
    BYTE n, res;


    if (cmd & 0x80) {	/* ACMD<n> is the command sequense of CMD55-CMD<n> */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    /* Select the card */
    CS_H(); rcvr_mmc();
    CS_L(); rcvr_mmc();

    /* Send a command packet */
    xmit_mmc(cmd);					/* Start + Command index */
    xmit_mmc((BYTE)(arg >> 24));	/* Argument[31..24] */
    xmit_mmc((BYTE)(arg >> 16));	/* Argument[23..16] */
    xmit_mmc((BYTE)(arg >> 8));		/* Argument[15..8] */
    xmit_mmc((BYTE)arg);			/* Argument[7..0] */
    n = 0x01;						/* Dummy CRC + Stop */
    if (cmd == CMD0) n = 0x95;		/* Valid CRC for CMD0(0) */
    if (cmd == CMD8) n = 0x87;		/* Valid CRC for CMD8(0x1AA) */
    xmit_mmc(n);

    /* Receive a command response */
    n = 10;								/* Wait for a valid response in timeout of 10 attempts */
    do {
        res = rcvr_mmc();
    } while ((res & 0x80) && --n);

    return res;			/* Return with the response value */
}



/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (void)
{
    BYTE n, cmd, ty, buf[4];
    UINT tmr;


    INIT_PORT();
    CS_H();
    skip_mmc(10);			/* Dummy clocks */

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {			/* Enter Idle state */
        if (send_cmd(CMD8, 0x1AA) == 1) {	/* SDv2 */
            for (n = 0; n < 4; n++) buf[n] = rcvr_mmc();	/* Get trailing return value of R7 resp */
            if (buf[2] == 0x01 && buf[3] == 0xAA) {			/* The card can work at vdd range of 2.7-3.6V */
                for (tmr = 1000; tmr; tmr--) {				/* Wait for leaving idle state (ACMD41 with HCS bit) */
                    if (send_cmd(ACMD41, 1UL << 30) == 0) break;
                    DLY_MS(1);
                }
                if (tmr && send_cmd(CMD58, 0) == 0) {		/* Check CCS bit in the OCR */
                    for (n = 0; n < 4; n++) buf[n] = rcvr_mmc();
                    ty = (buf[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;	/* SDv2 (HC or SC) */
                }
            }
        } else {							/* SDv1 or MMCv3 */
            if (send_cmd(ACMD41, 0) <= 1) 	{
                ty = CT_SD1; cmd = ACMD41;	/* SDv1 */
            } else {
                ty = CT_MMC; cmd = CMD1;	/* MMCv3 */
            }
            for (tmr = 1000; tmr; tmr--) {			/* Wait for leaving idle state */
                if (send_cmd(cmd, 0) == 0) break;
                DLY_MS(1);
            }
            if (!tmr || send_cmd(CMD16, 512) != 0)			/* Set R/W block length to 512 */
                ty = 0;
        }
    }
    CardType = ty;
    release_spi();

    return ty ? 0 : STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read partial sector                                                   */
/*-----------------------------------------------------------------------*/

DRESULT disk_readp (
        BYTE *buff,		/* Pointer to the read buffer (NULL:Read bytes are forwarded to the stream) */
        DWORD sector,	/* Sector number (LBA) */
        UINT offset,	/* Byte offset to read from (0..511) */
        UINT count		/* Number of bytes to read (ofs + cnt mus be <= 512) */
)
{
    DRESULT res;
    BYTE d;
    UINT bc, tmr;


    if (!(CardType & CT_BLOCK)) sector *= 512;	/* Convert to byte address if needed */

    res = RES_ERROR;
    if (send_cmd(CMD17, sector) == 0) {		/* READ_SINGLE_BLOCK */

        tmr = 100;
        do {							/* Wait for data packet in timeout of 100ms */
            DLY_MS(1);
            d = rcvr_mmc();
        } while (d == 0xFF && --tmr);

        if (d == 0xFE) {				/* A data packet arrived */
            bc = 514 - offset - count;

            /* Skip leading bytes */
            if (offset) skip_mmc(offset);

            /* Receive a part of the sector */
            /* Store data to the memory */
                do
                    *buff++ = rcvr_mmc();
                while (--count);

            }

            /* Skip trailing bytes and CRC */
            skip_mmc(bc);

            res = RES_OK;
        }
    }

    release_spi();

    return res;
}



/*-----------------------------------------------------------------------*/
/* Write partial sector                                                  */
/*-----------------------------------------------------------------------*/
#if _USE_WRITE

DRESULT disk_writep (
	const BYTE *buff,	/* Pointer to the bytes to be written (NULL:Initiate/Finalize sector write) */
	DWORD sc			/* Number of bytes to send, Sector number (LBA) or zero */
)
{
	DRESULT res;
	UINT bc, tmr;
	static UINT wc;


	res = RES_ERROR;

	if (buff) {		/* Send data bytes */
		bc = (UINT)sc;
		while (bc && wc) {		/* Send data bytes to the card */
			xmit_mmc(*buff++);
			wc--; bc--;
		}
		res = RES_OK;
	} else {
		if (sc) {	/* Initiate sector write transaction */
			if (!(CardType & CT_BLOCK)) sc *= 512;	/* Convert to byte address if needed */
			if (send_cmd(CMD24, sc) == 0) {			/* WRITE_SINGLE_BLOCK */
				xmit_mmc(0xFF); xmit_mmc(0xFE);		/* Data block header */
				wc = 512;							/* Set byte counter */
				res = RES_OK;
			}
		} else {	/* Finalize sector write transaction */
			bc = wc + 2;
			while (bc--) xmit_mmc(0);	/* Fill left bytes and CRC with zeros */
			if ((rcvr_mmc() & 0x1F) == 0x05) {	/* Receive data resp and wait for end of write process in timeout of 300ms */
				for (tmr = 1000; rcvr_mmc() != 0xFF && tmr; tmr--)	/* Wait for ready (max 1000ms) */
					DLY_MS(1);
				if (tmr) res = RES_OK;
			}
			release_spi();
		}
	}

	return res;
}
#endif