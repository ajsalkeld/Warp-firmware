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
#include <stdlib.h>

/*
 *	config.h needs to come first
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
#include "devPASCO2.h"


extern volatile WarpI2CDeviceState	devicePASCO2State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void initPASCO2(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
    devicePASCO2State.i2cAddress			= i2cAddress;
    devicePASCO2State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

    // Clear sticky bits
    writeSensorRegisterPASCO2(0x01, 0b00000111);

    // Check status
    readSensorRegisterPASCO2(0x01, 1);
    warpPrint("\r\n\t PASCO2 status: 0x%02x", devicePASCO2State.i2cBuffer[0]);

    // Put to idle mode
    writeSensorRegisterPASCO2(0x04, 0x00);

    // Delay 400ms
    OSA_TimeDelay(400);

    // Set pressure
    writeSensorRegisterPASCO2(0x0B, 0x03);
    writeSensorRegisterPASCO2(0x0C, 0xF5);

    return;
}

WarpStatus writeSensorRegisterPASCO2(uint8_t registerPointer, uint8_t payload)
{
    uint8_t		payloadByte[1], commandByte[1];
    i2c_status_t	status;

    switch (registerPointer)
    {
        case 0x01: /* Status */ case 0x02 ... 0x03: /* Measure rate */ case 0x04: /* Measure config */
        case 0x07: /* Measurements Status */ case 0x08: /* Interrupt Pin config */
        case 0x09 ... 0x0A: /* Alarm Thresh. */ case 0x0B ... 0x0C: /* Pressure compensation */
        case 0x0D ... 0x0E: /* Baseline reference */ case 0x0F: /* Scratch pad */ case 0x10: /* Soft reset */
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
                    .address = devicePASCO2State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    warpScaleSupplyVoltage(devicePASCO2State.operatingVoltageMillivolts);
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

WarpStatus readSensorRegisterPASCO2(uint8_t registerPointer, int numberOfBytes)
{
    i2c_status_t	status;


    USED(numberOfBytes);
    switch (registerPointer)
    {
        case 0x00: /* Product ID */
        case 0x01: /* Status */ case 0x02 ... 0x03: /* Measure rate */ case 0x04: /* Measure config */
        case 0x05 ... 0x06: /* CO2 PPM */
        case 0x07: /* Measurements Status */ case 0x08: /* Interrupt Pin config */
        case 0x09 ... 0x0A: /* Alarm Thresh. */ case 0x0B ... 0x0C: /* Pressure compensation */
        case 0x0D ... 0x0E: /* Baseline reference */ case 0x0F: /* Scratch pad */
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
                    .address = devicePASCO2State.i2cAddress,
                    .baudRate_kbps = gWarpI2cBaudRateKbps
            };

    warpScaleSupplyVoltage(devicePASCO2State.operatingVoltageMillivolts);
    warpEnableI2Cpins();

    status = I2C_DRV_MasterReceiveDataBlocking(
            0 /* I2C peripheral instance */,
            &slave,
            &registerPointer,
            1,
            (uint8_t *)devicePASCO2State.i2cBuffer,
            numberOfBytes,
            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

void printSensorDataPASCO2(bool hexModeFlag)
{
    int16_t		readSensorRegisterValueCombined;
    WarpStatus	i2cReadStatus;


    warpScaleSupplyVoltage(devicePASCO2State.operatingVoltageMillivolts);

    writeSensorRegisterPASCO2(0x01, 0b00000111);

    // TODO: Set pressure from BME280

    // Single measurement
    writeSensorRegisterPASCO2(0x04, 0x01);
    OSA_TimeDelay(1000);

    // Get CO2 PPM
    i2cReadStatus = readSensorRegisterPASCO2(0x05, 2);

    // Receive MSB,LSB
    readSensorRegisterValueCombined = (uint16_t) ( ( devicePASCO2State.i2cBuffer[0] << 8 ) | devicePASCO2State.i2cBuffer[1] );

    // Get status
    i2cReadStatus = readSensorRegisterPASCO2(0x01, 1);
    uint8_t CO2Status = devicePASCO2State.i2cBuffer[0];

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----, ----, ");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%04x, 0x%02x, ", readSensorRegisterValueCombined, CO2Status);
        }
        else
        {
            warpPrint(" %d, %d, ", readSensorRegisterValueCombined, CO2Status);
        }
    }
}