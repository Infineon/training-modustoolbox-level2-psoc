/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cybsp.h"
#include <stdio.h>


int main(void)
{
    cy_rslt_t result;

    // UART context variable
	cy_stc_scb_uart_context_t UART_context;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Configure and enable the UART peripheral */
	Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
	Cy_SCB_UART_Enable(UART_HW);

	// Initialize the AREF - block needed by ADC_HW ADC
	Cy_SysAnalog_Init(&AREF_config);
	// Initialize AREF
	Cy_SysAnalog_Enable();

	// Initialize the ADC
	Cy_SAR_Init(ADC_HW, &ADC_config);
	// Enable the ADC
	Cy_SAR_Enable(ADC_HW);

	int32_t ADCresult = 0; // ADC conversion result
	int32_t microVolts = 0; // Var to store voltage conversion of ADC result

    for (;;){
    	// Start a single conversion
		Cy_SAR_StartConvert(ADC_HW, CY_SAR_START_CONVERT_SINGLE_SHOT);
		if(Cy_SAR_IsEndConversion(ADC_HW, CY_SAR_WAIT_FOR_RESULT) == CY_SAR_SUCCESS){
			ADCresult = Cy_SAR_GetResult32(ADC_HW, 0);
			microVolts = Cy_SAR_CountsTo_uVolts(ADC_HW, 0, ADCresult);
		}
		char stringBuffer[20];
		sprintf(stringBuffer, "%ld\r\n", microVolts);
		Cy_SCB_UART_PutString(UART_HW, stringBuffer);
		Cy_SysLib_Delay(100);
    }
}

/* [] END OF FILE */
