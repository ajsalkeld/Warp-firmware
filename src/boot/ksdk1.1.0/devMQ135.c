/*
	Authored 2021. Andrew Salkeld.

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
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMQ135.h"

#define MQ135_CHANNEL 8u

#if FSL_FEATURE_ADC16_HAS_CALIBRATION
adc16_calibration_param_t AdcCalibraitionParam;
#endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //

adc16_user_config_t AdcUsrConfig;
adc16_chn_config_t ChnConfig;

void initMQ135( void )
{
    #if FSL_FEATURE_ADC16_HAS_CALIBRATION
    // Auto calibration. //
    ADC16_DRV_GetAutoCalibrationParam(0, &AdcCalibraitionParam);
    ADC16_DRV_SetCalibrationParam(0, &AdcCalibraitionParam);
    #endif // FSL_FEATURE_ADC16_HAS_CALIBRATION //

    // Initialize the ADC converter. //
    // First get default single-shot options.
    ADC16_DRV_StructInitUserConfigDefault(&AdcUsrConfig);

    // Set to 8-bit
    AdcUsrConfig.resolutionMode = kAdcResolutionBitOfSingleEndAs8;

    // Set ADC reference high to VDD = 3.0V
    AdcUsrConfig.refVoltSrcMode = kAdcRefVoltSrcOfVref;

    ADC16_DRV_Init(0, &AdcUsrConfig);

    // Set channel config
    ChnConfig.chnNum = MQ135_CHANNEL;
    ChnConfig.diffEnable = 0;
    ChnConfig.intEnable = 0;
    #if FSL_FEATURE_ADC16_HAS_MUX_SELECT
    ChnConfig.chnMux = kAdcChnMuxOfA;
    #endif // FSL_FEATURE_ADC16_HAS_MUX_SELECT //
}

uint8_t getReadingMQ135 (void)
{
    // Trigger the conversion with indicated channel’s configuration.
    ADC16_DRV_ConfigConvChn(0, 0, &ChnConfig);

    // Wait for the conversion to be done.
    ADC16_DRV_WaitConvDone(0, 0);

    // Fetch the conversion value and format it.
    uint8_t MQ135AdcValue = ADC16_DRV_GetConvValueRAW(0, 0);
    /*warpPrint("ADC16_DRV_GetConvValueRAW: 0x%X\t", MQ135AdcValue);
    warpPrint("ADC16_DRV_ConvRAWData: %ld\r\n",
           ADC16_DRV_ConvRAWData(MQ135AdcValue, false,
                                 kAdcResolutionBitOfSingleEndAs8) );*/

    uint8_t MQ135Converted = ADC16_DRV_ConvRAWData(MQ135AdcValue, false,
                                                   kAdcResolutionBitOfSingleEndAs8);

    ADC16_DRV_PauseConv(0, 0);

    return MQ135Converted;
}

void printSensorDataMQ135(bool hexModeFlag)
{
    uint8_t reading = getReadingMQ135();
    if (hexModeFlag)
    {
        warpPrint(" 0x%04x, ", reading);
    }
    else
    {
        warpPrint(" %d, ", reading);
    }
}