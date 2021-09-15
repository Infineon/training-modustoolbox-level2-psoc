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

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Global Vars
*******************************************************************************/
char buffer0[14] = "Hello World!\r\n";
char buffer1[14] = "DMA is cool!\r\n";

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures UART
*  2. Configures DMA
*  3. Sends text header to the UART serial terminal.
*  5. Waits in an infinite loop (for DMA or UART error interrupt)
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /////////////////////////////UART Config////////////////////////////////////////////

	// UART context variable
	cy_stc_scb_uart_context_t DEBUG_UART_context;

	/* Configure and enable the UART peripheral */
	Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
	Cy_SCB_UART_Enable(DEBUG_UART_HW);

	/////////////////////////////Timer Config////////////////////////////////////////////

	Cy_TCPWM_Counter_Init(MY_TIMER_HW, MY_TIMER_NUM, &MY_TIMER_config);
	Cy_TCPWM_Counter_Enable(MY_TIMER_HW, MY_TIMER_NUM);
	Cy_TCPWM_TriggerStart(MY_TIMER_HW, MY_TIMER_MASK);

	/////////////////////////////DMA Config////////////////////////////////////////////

	// Status variable for DMA initialization
	cy_en_dma_status_t dma_init_status;

	/* Initialize channel 0 descriptor 0 */
	dma_init_status = Cy_DMA_Descriptor_Init(&MY_DMA_Descriptor_0, &MY_DMA_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS){
		CY_ASSERT(0);
	}

	/* Initialize channel 1 descriptor 0 */
	dma_init_status = Cy_DMA_Descriptor_Init(&UART_DMA_Descriptor_0, &UART_DMA_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS){
		CY_ASSERT(0);
	}

	/* Set source and destination for channel 0 descriptor 0 */
	Cy_DMA_Descriptor_SetSrcAddress(&MY_DMA_Descriptor_0, (void *) buffer0);
	Cy_DMA_Descriptor_SetDstAddress(&MY_DMA_Descriptor_0, (void *) &(DEBUG_UART_HW->TX_FIFO_WR));

	/* Set source and destination for channel 1 descriptor 0 */
	Cy_DMA_Descriptor_SetSrcAddress(&UART_DMA_Descriptor_0, (void *) buffer1);
	Cy_DMA_Descriptor_SetDstAddress(&UART_DMA_Descriptor_0, (void *) &(DEBUG_UART_HW->TX_FIFO_WR));

	/* Initialize DMA channel 0 */
	dma_init_status = Cy_DMA_Channel_Init(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_channelConfig);
	CY_ASSERT(dma_init_status == CY_DMA_SUCCESS);

	/* Initialize DMA channel 1 */
	dma_init_status = Cy_DMA_Channel_Init(UART_DMA_HW, UART_DMA_CHANNEL, &UART_DMA_channelConfig);
	CY_ASSERT(dma_init_status == CY_DMA_SUCCESS);

	/* Set Descriptor for DMA Channel */
	Cy_DMA_Channel_SetDescriptor(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_Descriptor_0);

	/* Set Descriptor for DMA Channel */
	Cy_DMA_Channel_SetDescriptor(UART_DMA_HW, UART_DMA_CHANNEL, &UART_DMA_Descriptor_0);

	/* Enable DMA block */
	Cy_DMA_Enable(MY_DMA_HW);

	/* Enable Dma channel */
	Cy_DMA_Channel_Enable(MY_DMA_HW, MY_DMA_CHANNEL);

	/* Enable Dma channel */
	Cy_DMA_Channel_Enable(UART_DMA_HW, UART_DMA_CHANNEL);

	/////////////////////////////////////////////////////////////////////////

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "ModusToolbox Level 2 - PSoC - Print buffers using DMA\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");

	for (;;)
	{
		// The CPU can idle waiting for an interrupt.
	}
}
/* [] END OF FILE */
