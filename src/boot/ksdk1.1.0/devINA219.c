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
#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

    // Set a default config
    ina219_config.reset = 0;
    ina219_config.brng  = 1;        // 32V bus voltage range
    ina219_config.pga   = 0b11;     // +8 gain, range +/- 320mV
    ina219_config.badc  = 0b0011;   // 12-bit
    ina219_config.sadc  = 0b0011;   // ^^^
    ina219_config.mode  = 0b111;    // Continuous

    configureSensorINA219(ina219_config);

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t registerPointer, uint8_t * payload)
{
	uint8_t		payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (registerPointer)
	{
        case 0x00: /* Config */ case 0x05: /* Calibration */
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
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
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

    // Update last pointer
    lastPointer = registerPointer;

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(struct ConfigINA219 config)
{
	WarpStatus	i2cWriteStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cWriteStatus = writeSensorRegisterINA219(0x00 /* configuration pointer address */,
                                               (uint8_t *)&config /* payload: Disable FIFO */
							);

	return i2cWriteStatus;
}

WarpStatus
readSensorRegisterINA219(uint8_t registerPointer, int numberOfBytes)
{
	i2c_status_t	status;


	USED(numberOfBytes);
	switch (registerPointer)
	{
    case 0x00: /* Config */ case 0x01: /* Shunt V */ case 0x02: /* Bus V */ case 0x03: /* Power */
    case 0x04: /* Current */ case 0x05: /* Calibration */
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
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	warpEnableI2Cpins();

    if (registerPointer != lastPointer)
    {
        uint8_t     commandByte = registerPointer;

        status = I2C_DRV_MasterSendDataBlocking(
                0 /* I2C instance */,
                &slave,
                &commandByte,
                1,
                NULL, // No payload to send
                0,
                gWarpI2cTimeoutMilliseconds);

        if (status != kStatus_I2C_Success)
        {
            return kWarpStatusDeviceCommunicationFailed;
        }

        // Update last pointer
        lastPointer = registerPointer;
    }

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							NULL, // No command, receives from lastPointer
							0,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	int16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cReadStatus = readSensorRegisterINA219(0x04 /* Current register */, 2 /* numberOfBytes */);
    // Receive MSB,LSB
    readSensorRegisterValueCombined = (uint16_t) &deviceINA219State.i2cBuffer[0];

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
        if (hexModeFlag)
        {
            warpPrint(" 0x%04x,", readSensorRegisterValueCombined);
        }
        else
        {
            warpPrint(" %d,", readSensorRegisterValueCombined);
        }
	}
}