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

#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devBME280.h"

/*************************************************************************************************
 I2C Communication functions
 *************************************************************************************************/


extern volatile WarpI2CDeviceState	deviceBME280State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void initBME280(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
    deviceBME280State.i2cAddress			= i2cAddress;
    deviceBME280State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

    // TODO: Set config to forced mode

    WarpStatus i2cReadStatus = readSensorRegisterBME280(0x88, 25 /* numberOfBytes */);

    // dig_T1 ... 3
    dig_T1 = (uint16_t) &deviceBME280State.i2cBuffer[0];
    dig_T2 = (int16_t)  &deviceBME280State.i2cBuffer[2];
    dig_T3 = (int16_t)  &deviceBME280State.i2cBuffer[4];

    // dig_P1 ... 9
    dig_P1 = (uint16_t) &deviceBME280State.i2cBuffer[6];
    dig_P2 = (int16_t)  &deviceBME280State.i2cBuffer[8];
    dig_P3 = (int16_t)  &deviceBME280State.i2cBuffer[10];
    dig_P4 = (int16_t)  &deviceBME280State.i2cBuffer[12];
    dig_P5 = (int16_t)  &deviceBME280State.i2cBuffer[14];
    dig_P6 = (int16_t)  &deviceBME280State.i2cBuffer[16];
    dig_P7 = (int16_t)  &deviceBME280State.i2cBuffer[18];
    dig_P8 = (int16_t)  &deviceBME280State.i2cBuffer[20];
    dig_P9 = (int16_t)  &deviceBME280State.i2cBuffer[22];

    // dig_H1 ... 6
    dig_H1 = (uint8_t)  &deviceBME280State.i2cBuffer[24];

    i2cReadStatus = readSensorRegisterBME280(0xE1, 8 /* numberOfBytes */);

    dig_H2 = (int16_t)  &deviceBME280State.i2cBuffer[0];
    dig_H3 = (uint8_t)  &deviceBME280State.i2cBuffer[2];
    dig_H4 = (int16_t)  &deviceBME280State.i2cBuffer[3];
    dig_H5 = (int16_t)  &deviceBME280State.i2cBuffer[5];
    dig_H6 = (int8_t)   &deviceBME280State.i2cBuffer[7];

    return;
}

WarpStatus writeSensorRegisterBME280(uint8_t registerPointer, uint8_t * payload)
{
    uint8_t		payloadByte[2], commandByte[1];
    i2c_status_t	status;

    switch (registerPointer)
    {
        case 0xE0: /* Reset */ case 0xF2: /* ctrl_hum */ case 0xF4: /* ctrl_meas */ case 0xF5: /* config */
        {
            /* OK */
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
            {
                    .address = deviceBME280State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    warpScaleSupplyVoltage(deviceBME280State.operatingVoltageMillivolts);
    commandByte[0] = registerPointer;
    payloadByte[0] = payload[0];
    payloadByte[1] = payload[1];
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
            0 /* I2C instance */,
            &slave,
            commandByte,
            1,
            payloadByte,
            2,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus readSensorRegisterBME280(uint8_t registerPointer, int numberOfBytes)
{
    i2c_status_t	status;


    USED(numberOfBytes);
    switch (registerPointer)
    {
        case 0xF2: /* ctrl_hum */ case 0xF4: /* ctrl_meas */ case 0xF5: /* config */
        case 0xFE: /* hum_lsb */ case 0xFD: /* hum_msb */ case 0xFC: /* temp_xlsb */
        case 0xFB: /* temp_lsb */ case 0xFA: /* temp_msb */ case 0xF9: /* press_xlsb */
        case 0xF8: /* press_lsb */ case 0xF7: /* press_msb */ case 0xF3: /* status */
        case 0xD0: /* Chip ID */ case 0xE1 ... 0xF0: case 0x88 ... 0xA1: /* calib00 -> calib41 */
        {
            /* OK */
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
            {
                    .address = deviceBME280State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    warpScaleSupplyVoltage(deviceBME280State.operatingVoltageMillivolts);
    warpEnableI2Cpins();

    // First send write command with register
    status = I2C_DRV_MasterSendDataBlocking(
            0 /* I2C instance */,
            &slave,
            &registerPointer,
            1,
            NULL, // No payload to send
            0,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    status = I2C_DRV_MasterReceiveDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            NULL, // No register pointer, just receive.
            0,
            (uint8_t *)deviceBME280State.i2cBuffer,
            numberOfBytes,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

void printSensorDataBME280(bool hexModeFlag)
{
    WarpStatus	i2cReadStatus;

    warpScaleSupplyVoltage(deviceBME280State.operatingVoltageMillivolts);

    i2cReadStatus = readSensorRegisterBME280(0xF7, 8 /* numberOfBytes */);
    // Received press. MSB, LSB, XLSB; temp MSB, LSB, XLSB; hum. MSB, LSB

    int32_t receivedPressure = ( (int32_t) &deviceBME280State.i2cBuffer[0] ) >> 12;

    int32_t receivedTemp = ( (int32_t) &deviceBME280State.i2cBuffer[3] ) >> 12;

    int32_t receivedHum = (int32_t) ( (uint16_t) &deviceBME280State.i2cBuffer[6] );

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%08x, 0x%08x, 0x%04x,", receivedPressure, receivedTemp, receivedHum);
        }
        else
        {
            warpPrint(" %d, %d, %d,", BME280_compensate_P_int64(receivedPressure), BME280_compensate_T_int32(receivedTemp),
                      BME280_compensate_H_int32(receivedHum));
        }
    }
}

/*************************************************************************************************
 Conversion functions from BME280 databook.
 *************************************************************************************************/

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
BME280_S32_t t_fine;

BME280_S32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
{
    BME280_S32_t var1, var2, T;
    var1 = ((((adc_T>>3) - ((BME280_S32_t)dig_T1<<1))) * ((BME280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - ((BME280_S32_t)dig_T1)) * ((adc_T>>4) - ((BME280_S32_t)dig_T1))) >> 12) *
            ((BME280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BME280_U32_t BME280_compensate_P_int64(BME280_S32_t adc_P)
{
    BME280_S64_t var1, var2, p;
    var1 = ((BME280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BME280_S64_t)dig_P6;
    var2 = var2 + ((var1*(BME280_S64_t)dig_P5)<<17);
    var2 = var2 + (((BME280_S64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (BME280_S64_t)dig_P3)>>8) + ((var1 * (BME280_S64_t)dig_P2)<<12);
    var1 = (((((BME280_S64_t)1)<<47)+var1))*((BME280_S64_t)dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576-adc_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((BME280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((BME280_S64_t)dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((BME280_S64_t)dig_P7)<<4);
    return (BME280_U32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
BME280_U32_t BME280_compensate_H_int32(BME280_S32_t adc_H)
{
    BME280_S32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((BME280_S32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((BME280_S32_t)dig_H4) << 20) - (((BME280_S32_t)dig_H5) * v_x1_u32r)) +
            ((BME280_S32_t)16384)) >> 15) * (((((((v_x1_u32r * ((BME280_S32_t)dig_H6)) >> 10) * (((v_x1_u32r *
                    ((BME280_S32_t)dig_H3)) >> 11) + ((BME280_S32_t)32768))) >> 10) + ((BME280_S32_t)2097152)) *
                                              ((BME280_S32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((BME280_S32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (BME280_U32_t)(v_x1_u32r>>12);
}