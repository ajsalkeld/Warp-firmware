/*
	Authored 2021. Andrew Salkeld. Based on 2016-2018 Phillip Stanley-Marbell.

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

#ifndef WARP_FIRMWARE_DEVBME280_H
#define WARP_FIRMWARE_DEVBME280_H

#include <stdint.h>

typedef struct
{
    uint8_t     status;
    int32_t     temp;       // 0.01 degC
    uint32_t    pressure;   // 0.01 Pa
    uint32_t    humidity;   // Q22.10 (i.e. >> 10 for integer)
} devBME280Results;

// I2C Comms
WarpStatus readSensorRegisterBME280(uint8_t registerPointer, int numberOfBytes);

WarpStatus writeSensorRegisterBME280(uint8_t registerPointer, uint8_t payload);

void initBME280(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

WarpStatus getReadingsBME280 (devBME280Results * results_ptr, bool hexModeFlag);

#if (WARP_DEBUG_INTERFACE)
void printSensorDataBME280(bool hexModeFlag);
#else

#endif

#define triggerBME280(void) writeSensorRegisterBME280(0xF4, 0b00100101)

#define BME280_CONCAT_BYTES(msb, lsb)             (((uint16_t)msb << 8) | (uint16_t)lsb)

// Conversions

typedef int32_t     BME280_S32_t;
typedef uint32_t    BME280_U32_t;
typedef int64_t     BME280_S64_t;

extern BME280_S32_t t_fine;

struct devBME280CalibData
{
    uint16_t    dig_T1;     // 0x88 ...
    int16_t     dig_T2;
    int16_t     dig_T3;

    uint16_t    dig_P1;
    int16_t     dig_P2;
    int16_t     dig_P3;
    int16_t     dig_P4;
    int16_t     dig_P5;
    int16_t     dig_P6;
    int16_t     dig_P7;
    int16_t     dig_P8;
    int16_t     dig_P9;

    uint8_t     dig_H1;     // 0xE1
    int16_t     dig_H2;
    uint8_t     dig_H3;
    int16_t     dig_H4;
    int16_t     dig_H5;
    int8_t      dig_H6;
} devBME280_calib_data;

// Conversion functions
int32_t     BME280_compensate_temperature(int32_t adc_T);

uint32_t    BME280_compensate_pressure(int32_t adc_P);

uint32_t    BME280_compensate_humidity(int32_t adc_H);

#endif //WARP_FIRMWARE_DEVBME280_H
