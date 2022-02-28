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

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_INT_PRIORITY     (3u)
#define DMA_IRQ               (cpuss_interrupt_dma_IRQn)
#define DMA_INT_PRIORITY      (3u)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void Isr_UART(void);
void Isr_DMA(void);

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

    /* Enable global interrupts */
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

	/////////////////////////////DMA Config////////////////////////////////////////////

	char buffer;

	// DMA initialization status variable
	cy_en_dmac_status_t dmac_init_status;

	/* DMA interrupt initialization structure */
	cy_stc_sysint_t DMA_INT_cfg =
	{
		.intrSrc      = (IRQn_Type)DMA_IRQ,
		.intrPriority = DMA_INT_PRIORITY,
	};

	/* Initialize and enable the DMA interrupt */
	Cy_SysInt_Init(&DMA_INT_cfg, &Isr_DMA);
	NVIC_EnableIRQ(DMA_INT_cfg.intrSrc);

	/* Enable interrupt for DMA channel */
	Cy_DMAC_SetInterruptMask(RX_DMA_HW,  CY_DMAC_INTR_CHAN_0);

	/* Enable interrupt for DMA channel */
	Cy_DMAC_SetInterruptMask(RX_DMA_HW,  CY_DMAC_INTR_CHAN_1);

	// Initialize the RX PING descriptor
	dmac_init_status = Cy_DMAC_Descriptor_Init(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, &RX_DMA_ping_config);
	if(dmac_init_status != CY_DMAC_SUCCESS){
		CY_ASSERT(0);
	}

	// Initialize the TX PING descriptor
	dmac_init_status = Cy_DMAC_Descriptor_Init(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, &TX_DMA_ping_config);
	if(dmac_init_status != CY_DMAC_SUCCESS){
		CY_ASSERT(0);
	}

	/* Initialize RX Dma channel */
	dmac_init_status = Cy_DMAC_Channel_Init(RX_DMA_HW, RX_DMA_CHANNEL, &RX_DMA_channel_config);
	CY_ASSERT(dmac_init_status == CY_DMAC_SUCCESS);

	/* Initialize TX Dma channel */
	dmac_init_status = Cy_DMAC_Channel_Init(TX_DMA_HW, TX_DMA_CHANNEL, &TX_DMA_channel_config);
	CY_ASSERT(dmac_init_status == CY_DMAC_SUCCESS);

	/* Set source and destination for RX PING descriptor */
	Cy_DMAC_Descriptor_SetSrcAddress(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, (void *) &(DEBUG_UART_HW->RX_FIFO_RD));
	Cy_DMAC_Descriptor_SetDstAddress(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, (void *) &buffer);

	/* Set source and destination for TX PING descriptor */
	Cy_DMAC_Descriptor_SetSrcAddress(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, (void *) &buffer);
	Cy_DMAC_Descriptor_SetDstAddress(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, (void *) &(DEBUG_UART_HW->TX_FIFO_WR));

	/* Validate the RX PING descriptor */
	Cy_DMAC_Descriptor_SetState(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, true);

	/* Validate the TX PING descriptor */
	Cy_DMAC_Descriptor_SetState(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING, true);

	/* Set PING descriptor as current descriptor for RX Dma channel  */
	Cy_DMAC_Channel_SetCurrentDescriptor(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING);

	/* Set PING descriptor as current descriptor for TX Dma channel  */
	Cy_DMAC_Channel_SetCurrentDescriptor(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING);

	/* Enable DMAC block */
	Cy_DMAC_Enable(RX_DMA_HW);

	/* Enable RX Dma channel */
	Cy_DMAC_Channel_Enable(RX_DMA_HW, RX_DMA_CHANNEL);

	/* Enable TX Dma channel */
	Cy_DMAC_Channel_Enable(TX_DMA_HW, TX_DMA_CHANNEL);

	/////////////////////////////////////////////////////////////////////////

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "ModusToolbox Level 2 - PSoC - UART echo using DMA channel chaining\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, ">> Start typing to see the echo on the screen \r\n\n");

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
	/* If channel 0 triggered the interrupt */
	if ((CY_DMAC_INTR_CHAN_0 & Cy_DMAC_GetInterruptStatusMasked(RX_DMA_HW)) > 0U){
		Cy_DMAC_ClearInterrupt(RX_DMA_HW, CY_DMAC_INTR_CHAN_0); // Clear the interrupt
		// Make sure the transfer completed successfully
		if(Cy_DMAC_Descriptor_GetResponse(RX_DMA_HW, RX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING) != CY_DMAC_DONE){
			Cy_SCB_UART_PutString(DEBUG_UART_HW, "DMA Error Occurred. Halting Execution.\r\n");
			CY_ASSERT(0);
		}
	}
	/* If channel 1 triggered the interrupt */
	if ((CY_DMAC_INTR_CHAN_1 & Cy_DMAC_GetInterruptStatusMasked(TX_DMA_HW)) > 0U){
		Cy_DMAC_ClearInterrupt(TX_DMA_HW, CY_DMAC_INTR_CHAN_1); // Clear the interrupt
		// Make sure the transfer completed successfully
		if(Cy_DMAC_Descriptor_GetResponse(TX_DMA_HW, TX_DMA_CHANNEL, CY_DMAC_DESCRIPTOR_PING) != CY_DMAC_DONE){
			Cy_SCB_UART_PutString(DEBUG_UART_HW, "DMA Error Occurred. Halting Execution.\r\n");
			CY_ASSERT(0);
		}
	}
}


/* [] END OF FILE */
