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
#include "book.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_INT_PRIORITY     (7u)
#define DMA_INT_PRIORITY      (7u)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void Isr_UART(void);
void Isr_DMA(void);

/*******************************************************************************
* Global Vars
*******************************************************************************/
const struct book book1 = {
	.title = "Moby Dick",
	.author = "Herman Melville",
	.pageCount = 378
};

const struct book book2 = {
	.title = "Around the World in 80 Days",
	.author = "Jules Verne",
	.pageCount = 188
};

const struct book book3 = {
	.title = "The Adventures of Tom Sawyer",
	.author = "Mark Twain",
	.pageCount = 274
};

const struct book book4 = {
	.title = "1984",
	.author = "George Orwell",
	.pageCount = 328
};

const struct book book5 = {
	.title = "Frankenstein",
	.author = "Mary Shelley",
	.pageCount = 280
};

const struct book bookArray[5] = {book1, book2, book3, book4, book5}; // Array of book structs stored in internal flash

struct book bookArrayCopied[5]; // Array in ram to copy the flash array to

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

	/* UART interrupt initialization structure  */
	cy_stc_sysint_t DEBUG_UART_INT_cfg =
	 {
		 .intrSrc      = DEBUG_UART_IRQ,
		 .intrPriority = UART_INT_PRIORITY
	 };

	/* Initialize and enable the UART interrupt */
	Cy_SysInt_Init(&DEBUG_UART_INT_cfg, &Isr_UART);
	NVIC_EnableIRQ(DEBUG_UART_INT_cfg.intrSrc);

	/* Configure and enable the UART peripheral */
	Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
	Cy_SCB_UART_Enable(DEBUG_UART_HW);

	/////////////////////////////Timer Config////////////////////////////////////////////
	Cy_TCPWM_Counter_Init(MY_TIMER_HW, MY_TIMER_NUM, &MY_TIMER_config);
	Cy_TCPWM_Counter_Enable(MY_TIMER_HW, MY_TIMER_NUM);
	Cy_TCPWM_TriggerStart_Single(MY_TIMER_HW, MY_TIMER_NUM);

	/////////////////////////////DMA Config////////////////////////////////////////////
	// Status variable for DMA initialization
	cy_en_dma_status_t dma_init_status;

	/* DMA interrupt initialization structure */
	cy_stc_sysint_t DMA_INT_cfg =
	{
		.intrSrc      = (IRQn_Type)cpuss_interrupts_dmac_0_IRQn,
		.intrPriority = DMA_INT_PRIORITY,
	};

	/* Initialize and enable the DMA interrupt */
	Cy_SysInt_Init(&DMA_INT_cfg, &Isr_DMA);
	NVIC_EnableIRQ(DMA_INT_cfg.intrSrc);

	/* Enable interrupt for DMA channel */
	Cy_DMAC_Channel_SetInterruptMask(MY_DMA_HW, MY_DMA_CHANNEL, CY_DMAC_INTR_MASK);

	/* Initialize descriptor 0 */
	dma_init_status = Cy_DMAC_Descriptor_Init(&MY_DMA_Descriptor_0, &MY_DMA_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS){
		CY_ASSERT(0);
	}

	// TODO Set source and destination for descriptor 0
	Cy_DMAC_Descriptor_SetSrcAddress(&MY_DMA_Descriptor_0, (void *) );
	Cy_DMAC_Descriptor_SetDstAddress(&MY_DMA_Descriptor_0, (void *) );

	/* Initialize DMA channel */
	dma_init_status = Cy_DMAC_Channel_Init(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_channelConfig);
	CY_ASSERT(dma_init_status == CY_DMA_SUCCESS);

	/* Set Descriptor for DMA Channel */
	Cy_DMAC_Channel_SetDescriptor(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_Descriptor_0);

	/* Enable DMA block */
	Cy_DMAC_Enable(MY_DMA_HW);

	/* Enable Dma channel */
	Cy_DMAC_Channel_Enable(MY_DMA_HW, MY_DMA_CHANNEL);

	/////////////////////////////////////////////////////////////////////////
	
	Cy_SysLib_Delay(10); // Allow for initialization to finish

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "ModusToolbox Level 2 - PSoC - Copying an array of structs\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");

	for (;;)
	{
		// The CPU can idle waiting for an interrupt.
	}
}

/*******************************************************************************
* Function Name: Isr_UART
********************************************************************************
*
* Summary:
* Handles UART underflow and overflow conditions. This conditions must never
* occur.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void Isr_UART(void)
{
    uint32_t rx_intr_src;
    uint32_t tx_intr_src;

    /* Get RX interrupt sources */
    rx_intr_src =  Cy_SCB_UART_GetRxFifoStatus(DEBUG_UART_HW);
    Cy_SCB_UART_ClearRxFifoStatus(DEBUG_UART_HW, rx_intr_src);

    /* Get TX interrupt sources */
    tx_intr_src =  Cy_SCB_UART_GetTxFifoStatus(DEBUG_UART_HW);
    Cy_SCB_UART_ClearTxFifoStatus(DEBUG_UART_HW, tx_intr_src);

    Cy_SCB_UART_PutString(DEBUG_UART_HW, "UART Error Occurred. Halting Execution.\r\n");
    CY_ASSERT(0);
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
	if(!(CY_DMAC_INTR_COMPLETION == Cy_DMAC_Channel_GetInterruptStatusMasked(MY_DMA_HW, MY_DMA_CHANNEL))){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "DMA Error Occurred. Halting Execution.\r\n");
		CY_ASSERT(0);
	}

	// Print all of the copied book info
	for(int i = 0; i < 5; i++){
		printBookInfo(DEBUG_UART_HW, bookArrayCopied[i]);
	}

	/* Clear Dma channel interrupt */
	Cy_DMAC_Channel_ClearInterrupt(MY_DMA_HW, MY_DMA_CHANNEL, Cy_DMAC_Channel_GetInterruptStatusMasked(MY_DMA_HW, MY_DMA_CHANNEL));
}
/* [] END OF FILE */
