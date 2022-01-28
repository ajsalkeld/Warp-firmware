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

    // Reset
    writeSensorRegisterBME280(0xE0, 0xB6);

    // config: Set filter off
    writeSensorRegisterBME280(0xF5, 0b0000000);

    // ctrl_hum: Set humidity oversampling to x1
    writeSensorRegisterBME280(0xF2, 0b00000001);

    // ctrl_meas: Set temp and pressure oversamp to x1; mode to sleep.
    writeSensorRegisterBME280(0xF4, 0b00100100);

    // Get Calibration data
    readSensorRegisterBME280(0x88, 26 /* numberOfBytes */);

    // dig_T1 ... 3
    devBME280_calib_data.dig_T1 =           BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[1], deviceBME280State.i2cBuffer[0]);
    devBME280_calib_data.dig_T2 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[3], deviceBME280State.i2cBuffer[2]);
    devBME280_calib_data.dig_T3 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[5], deviceBME280State.i2cBuffer[4]);

    // dig_P1 ... 9
    devBME280_calib_data.dig_P1 =           BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[7], deviceBME280State.i2cBuffer[6]);
    devBME280_calib_data.dig_P2 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[9], deviceBME280State.i2cBuffer[8]);
    devBME280_calib_data.dig_P3 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[11], deviceBME280State.i2cBuffer[10]);
    devBME280_calib_data.dig_P4 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[13], deviceBME280State.i2cBuffer[12]);
    devBME280_calib_data.dig_P5 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[15], deviceBME280State.i2cBuffer[14]);
    devBME280_calib_data.dig_P6 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[17], deviceBME280State.i2cBuffer[16]);
    devBME280_calib_data.dig_P7 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[19], deviceBME280State.i2cBuffer[18]);
    devBME280_calib_data.dig_P8 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[21], deviceBME280State.i2cBuffer[20]);
    devBME280_calib_data.dig_P9 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[23], deviceBME280State.i2cBuffer[22]);


    // dig_H1 ... 6
    devBME280_calib_data.dig_H1 =           deviceBME280State.i2cBuffer[25];

    // Remaining dig_Hs in E1 ... E8
    readSensorRegisterBME280(0xE1, 7);

    devBME280_calib_data.dig_H2 = (int16_t) BME280_CONCAT_BYTES(deviceBME280State.i2cBuffer[1], deviceBME280State.i2cBuffer[0]);
    devBME280_calib_data.dig_H3 =           deviceBME280State.i2cBuffer[2];
    devBME280_calib_data.dig_H4 = (int16_t)  ( ( (uint8_t) deviceBME280State.i2cBuffer[3] ) << 4  | ( (uint8_t) deviceBME280State.i2cBuffer[4] & 0b00001111 ) );
    devBME280_calib_data.dig_H5 = (int16_t)  ( ( ( (uint8_t) deviceBME280State.i2cBuffer[4] & 0b11110000 ) >> 4 ) | ( (uint8_t) deviceBME280State.i2cBuffer[5] ) << 4  );
    devBME280_calib_data.dig_H6 = (int8_t)   deviceBME280State.i2cBuffer[6];

    return;
}

WarpStatus writeSensorRegisterBME280(uint8_t registerPointer, uint8_t payload)
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

    //warpScaleSupplyVoltage(deviceBME280State.operatingVoltageMillivolts);
    commandByte[0] = registerPointer;
    payloadByte[0] = payload;
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
            0 /* I2C instance */,
            &slave,
            commandByte,
            1,
            payloadByte,
            1,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus readSensorRegisterBME280(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t		cmdBuf[1] = {0xFF};
    i2c_status_t	status;


    USED(numberOfBytes);

    switch (deviceRegister)
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

    cmdBuf[0] = deviceRegister;

    //warpScaleSupplyVoltage(deviceBME280State.operatingVoltageMillivolts);
    warpEnableI2Cpins();
    status = I2C_DRV_MasterReceiveDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            cmdBuf,
            1,
            (uint8_t *)deviceBME280State.i2cBuffer,
            numberOfBytes,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus getReadingsBME280 (devBME280Results * results_ptr, bool hexModeFlag)
{
    // Trigger recording
    triggerBME280();

    WarpStatus i2cReadStatus1 = readSensorRegisterBME280(0xF3, 1);
    results_ptr->status = deviceBME280State.i2cBuffer[0];

    // Now read values
    WarpStatus i2cReadStatus2 = readSensorRegisterBME280(0xF7, 8);

    // Received press. MSB, LSB, XLSB; temp MSB, LSB, XLSB; hum. MSB, LSB. Positive but store signed.

    int32_t receivedPressure = ( ((uint32_t) deviceBME280State.i2cBuffer[0]) << 12 ) | ( deviceBME280State.i2cBuffer[1] << 4 )
                               | ( (deviceBME280State.i2cBuffer[2] & 0xF0) >> 4 );

    int32_t receivedTemp = ( ((uint32_t) deviceBME280State.i2cBuffer[3]) << 12 ) | ( deviceBME280State.i2cBuffer[4] << 4 )
                           | ( (deviceBME280State.i2cBuffer[5] & 0xF0) >> 4 );

    int32_t receivedHum = ( deviceBME280State.i2cBuffer[6] << 8 ) | ( deviceBME280State.i2cBuffer[7] );

    if (hexModeFlag)
    {
        results_ptr->temp       = receivedTemp;
        results_ptr->pressure   = receivedPressure;
        results_ptr->humidity   = receivedHum;
    }
    else
    {
        results_ptr->temp = BME280_compensate_temperature(receivedTemp);

        results_ptr->pressure = BME280_compensate_pressure(receivedPressure);

        results_ptr->humidity = BME280_compensate_humidity(receivedHum);
    }

    return (i2cReadStatus1 || i2cReadStatus2);
}

#if (WARP_DEBUG_INTERFACE)
void printSensorDataBME280(bool hexModeFlag)
{
    WarpStatus	i2cReadStatus;

    devBME280Results readings;

    i2cReadStatus = getReadingsBME280(&readings, hexModeFlag);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%08x, 0x%08x, 0x%04x, 0x%04x, ", readings.temp, readings.pressure, readings.humidity, readings.status);
        }
        else
        {
            warpPrint(" %d, %d, %d, 0x%04x, ", readings.temp, readings.pressure, readings.humidity >> 10, readings.status); // Always compensate T first
        }
    }
}
#endif


/*************************************************************************************************
 Conversion functions adapted from Bosch API. (https://github.com/BoschSensortec/BME280_driver)
 Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.

    BSD-3-Clause

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
    STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
    IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************************************/

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;

int32_t BME280_compensate_temperature(int32_t adc_T)
{
    int32_t var1;
    int32_t var2;
    int32_t temperature;
    int32_t temperature_min = -4000;
    int32_t temperature_max = 8500;

    var1 = (int32_t)((adc_T / 8) - ((int32_t)devBME280_calib_data.dig_T1 * 2));
    var1 = (var1 * ((int32_t)devBME280_calib_data.dig_T2)) / 2048;
    var2 = (int32_t)((adc_T / 16) - ((int32_t)devBME280_calib_data.dig_T1));
    var2 = (((var2 * var2) / 4096) * ((int32_t)devBME280_calib_data.dig_T3)) / 16384;
    t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) / 256;

    if (temperature < temperature_min)
    {
        temperature = temperature_min;
    }
    else if (temperature > temperature_max)
    {
        temperature = temperature_max;
    }

    return temperature;
}

// Returns pressure in 0.01 Pa
uint32_t BME280_compensate_pressure(int32_t adc_P)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int64_t var4;
    uint32_t pressure;
    uint32_t pressure_min = 3000000;
    uint32_t pressure_max = 11000000;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)devBME280_calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)devBME280_calib_data.dig_P5) * 131072);
    var2 = var2 + (((int64_t)devBME280_calib_data.dig_P4) * 34359738368);
    var1 = ((var1 * var1 * (int64_t)devBME280_calib_data.dig_P3) / 256) + ((var1 * ((int64_t)devBME280_calib_data.dig_P2) * 4096));
    var3 = ((int64_t)1) * 140737488355328;
    var1 = (var3 + var1) * ((int64_t)devBME280_calib_data.dig_P1) / 8589934592;

    /* To avoid divide by zero exception */
    if (var1 != 0)
    {
        var4 = 1048576 - adc_P;
        var4 = (((var4 * INT64_C(2147483648)) - var2) * 3125) / var1;
        var1 = (((int64_t)devBME280_calib_data.dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
        var2 = (((int64_t)devBME280_calib_data.dig_P8) * var4) / 524288;
        var4 = ((var4 + var1 + var2) / 256) + (((int64_t)devBME280_calib_data.dig_P7) * 16);
        pressure = (uint32_t)(((var4 / 2) * 100) / 128);

        if (pressure < pressure_min)
        {
            pressure = pressure_min;
        }
        else if (pressure > pressure_max)
        {
            pressure = pressure_max;
        }
    }
    else
    {
        pressure = pressure_min;
    }

    return pressure;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280_compensate_humidity(int32_t adc_H)
{
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    uint32_t humidity;
    uint32_t humidity_max = 102400;

    var1 = t_fine - ((int32_t)76800);
    var2 = (int32_t)(adc_H * 16384);
    var3 = (int32_t)(((int32_t)devBME280_calib_data.dig_H4) * 1048576);
    var4 = ((int32_t)devBME280_calib_data.dig_H5) * var1;
    var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
    var2 = (var1 * ((int32_t)devBME280_calib_data.dig_H6)) / 1024;
    var3 = (var1 * ((int32_t)devBME280_calib_data.dig_H3)) / 2048;
    var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
    var2 = ((var4 * ((int32_t)devBME280_calib_data.dig_H2)) + 8192) / 16384;
    var3 = var5 * var2;
    var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
    var5 = var3 - ((var4 * ((int32_t)devBME280_calib_data.dig_H1)) / 16);
    var5 = (var5 < 0 ? 0 : var5);
    var5 = (var5 > 419430400 ? 419430400 : var5);
    humidity = (uint32_t)(var5 / 4096);

    if (humidity > humidity_max)
    {
        humidity = humidity_max;
    }

    return humidity;
}