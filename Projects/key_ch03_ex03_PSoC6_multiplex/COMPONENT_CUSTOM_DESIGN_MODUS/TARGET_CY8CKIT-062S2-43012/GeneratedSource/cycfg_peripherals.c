/*******************************************************************************
* File Name: cycfg_peripherals.c
*
* Description:
* Peripheral Hardware Block configuration
* This file was automatically generated and should not be modified.
* Tools Package 2.3.0.4276
* mtb-pdl-cat1 2.2.0.7891
* personalities 4.0.0.0
* udd 3.0.0.936
*
********************************************************************************
* Copyright 2021 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#include "cycfg_peripherals.h"

cy_stc_csd_context_t cy_csd_0_context = 
{
	.lockKey = CY_CSD_NONE_KEY,
};
static const cy_stc_csdadc_ch_pin_t CYBSP_CSD_csdadc_channel_list[] = 
{
	[0] = {
            .ioPcPtr = GPIO_PRT10, 
            .pin = 6u, 
          },
};
const cy_stc_csdadc_config_t CYBSP_CSD_csdadc_config = 
{
	.ptrPinList = &CYBSP_CSD_csdadc_channel_list[0u],
	.base = CYBSP_CSD_HW,
	.cpuClk = 100000000u,
	.periClk = 100000000u,
	.vref = -1,
	.vdda = 3300u,
	.calibrInterval = 30u,
	.range = CY_CSDADC_RANGE_VDDA,
	.resolution = CY_CSDADC_RESOLUTION_10BIT,
	.periDivTyp = CY_SYSCLK_DIV_8_BIT,
	.numChannels = 1u,
	.idac = 31u,
	.operClkDivider = 2u,
	.azTime = 5u,
	.acqTime = 10u,
	.csdInitTime = 25u,
	.idacCalibrationEn = 0u,
	.periDivInd = 0u,
	.csdCxtPtr = &cy_csd_0_context,
};


void init_cycfg_peripherals(void)
{
	Cy_SysClk_PeriphAssignDivider(PCLK_CSD_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
}
