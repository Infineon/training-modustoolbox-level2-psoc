/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC4  Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) (2020), Cypress Semiconductor Corporation. All rights reserved.
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

volatile bool pressedFlag = false;

#define PORT3_INTR_MASK  (0x00000001UL << 3)

void Interrupt_Handler_Port3(void){
	// Get interrupt cause
	uint32_t intrSrc = Cy_GPIO_GetInterruptCause();
	/* Check if the interrupt was from port 3 */
	if(PORT3_INTR_MASK == (intrSrc & PORT3_INTR_MASK)){
		/* Clear the P3.7 interrupt */
		Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
		// Toggle LED
		Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM);
		pressedFlag = true;
	}
}

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

    /* Enable global interrupts */
    __enable_irq();

    // Interrupt config structure
    cy_stc_sysint_t intrCfg =
	{
		/*.intrSrc =*/ ioss_interrupts_gpio_3_IRQn, /* Interrupt source is GPIO port 3 interrupt */
		/*.intrPriority =*/ 3UL                     /* Interrupt priority is 7 */
	};

    /* Initialize the interrupt with vector at Interrupt_Handler_Port3() */
	Cy_SysInt_Init(&intrCfg, &Interrupt_Handler_Port3);
	// Send the button through the glitch filter
	Cy_GPIO_SetFilter(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
	// Falling edge interrupt
	Cy_GPIO_SetInterruptEdge(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM, CY_GPIO_INTR_FALLING);
	/* Enable the interrupt */
	NVIC_EnableIRQ(intrCfg.intrSrc);

	/* Configure and enable the UART peripheral */
	Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
	Cy_SCB_UART_Enable(UART_HW);

	uint32_t pressCounter = 0;

    for (;;){
    	// If the button was pressed, increment pressCounter and send the value over UART
    	if(pressedFlag){
    		if(pressCounter == 9){
    			pressCounter = 0;
    		}
    		else{
    			pressCounter++;
    		}
    		uint8_t dataBuffer = 0;
    		dataBuffer = pressCounter + '0';
    		Cy_SCB_UART_Put(UART_HW, dataBuffer);
    		pressedFlag = false;
    	}
    }
}

/* [] END OF FILE */