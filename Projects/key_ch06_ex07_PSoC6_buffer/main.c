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
* Macros
*******************************************************************************/
#define DMA_INT_PRIORITY      (7u)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void Isr_DMA(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
char buffer[5];

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

	/////////////////////////////DMA Config////////////////////////////////////////////

	// Status variable for DMA initialization
	cy_en_dma_status_t dma_init_status;

	/* DMA interrupt initialization structure */
	cy_stc_sysint_t DMA_INT_cfg =
	{
		.intrSrc      = (IRQn_Type)UART_DMA_IRQ,
		.intrPriority = DMA_INT_PRIORITY,
	};

	/* Initialize and enable the DMA interrupt */
	Cy_SysInt_Init(&DMA_INT_cfg, &Isr_DMA);
	NVIC_EnableIRQ(DMA_INT_cfg.intrSrc);

	/* Enable interrupt for DMA channel */
	Cy_DMA_Channel_SetInterruptMask(UART_DMA_HW, UART_DMA_CHANNEL, CY_DMA_INTR_MASK);

	/* Initialize descriptor 0 */
	dma_init_status = Cy_DMA_Descriptor_Init(&UART_DMA_Descriptor_0, &UART_DMA_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS){
		CY_ASSERT(0);
	}

	/* Set source and destination for descriptor 0 */
	Cy_DMA_Descriptor_SetSrcAddress(&UART_DMA_Descriptor_0, (void *) &(DEBUG_UART_HW->RX_FIFO_RD));
	Cy_DMA_Descriptor_SetDstAddress(&UART_DMA_Descriptor_0, (void *) buffer);

	/* Initialize DMA channel */
	dma_init_status = Cy_DMA_Channel_Init(UART_DMA_HW, UART_DMA_CHANNEL, &UART_DMA_channelConfig);
	CY_ASSERT(dma_init_status == CY_DMA_SUCCESS);

	/* Set Descriptor for DMA Channel */
	Cy_DMA_Channel_SetDescriptor(UART_DMA_HW, UART_DMA_CHANNEL, &UART_DMA_Descriptor_0);

	/* Enable DMA block */
	Cy_DMA_Enable(UART_DMA_HW);

	/* Enable Dma channel */
	Cy_DMA_Channel_Enable(UART_DMA_HW, UART_DMA_CHANNEL);

	/////////////////////////////////////////////////////////////////////////

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "ModusToolbox Level 2 - PSoC - UART capitalize buffer using DMA\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, ">> Start typing to see the echo on the screen \r\n\n");


	for (;;)
	{
		// The CPU can idle waiting for an interrupt.
	}
}

/*******************************************************************************
* Function Name: Isr_DMA
********************************************************************************
*
* Summary:
*  Handles Dma descriptor completion interrupt source
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Isr_DMA(void)
{
	/* Check if the Dma channel response is successful for current transfer */
	if(!(CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(UART_DMA_HW, UART_DMA_CHANNEL))){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "DMA Error Occurred. Halting Execution.\r\n");
		CY_ASSERT(0);
	}

	// Print the received characters in reverse order to the terminal
	char reversedBuffer[5];
	for(int i = 0; i < 5; i++){
		reversedBuffer[i] = buffer[4 - i];
	}
	Cy_SCB_UART_PutArray(DEBUG_UART_HW, (void*)reversedBuffer, 5);

	/* Clear Dma channel interrupt */
	Cy_DMA_Channel_ClearInterrupt(UART_DMA_HW, UART_DMA_CHANNEL);
}
/* [] END OF FILE */
