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
#include "cy_serial_flash_qspi.h"
#include "cycfg_qspi_memslot.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_INT_PRIORITY     	(7u)
#define DMA_INT_PRIORITY      	(7u)
#define MEM_SLOT_NUM          	(0u) // QSPI configuration slot number
#define QSPI_BUS_FREQUENCY_HZ   (50000000lu)
#define MESSAGE_LENGTH			(14)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void Isr_DMA(void);

/*******************************************************************************
* Global Vars
*******************************************************************************/
uint32_t startOfFlashGlobal = 0x18000000; // Start of external flash memory in global address space
uint32_t startOfFlash = 0x00; // Start of flash memory relative to beginning of flash memory

char buffer[MESSAGE_LENGTH] = "Hello World!\r\n"; // Buffer to write to flash memory
char buffer_r[MESSAGE_LENGTH] = ""; // Buffer to read into

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

    /////////////////////////////Setup external flash////////////////////////////////////////////

    // Initialize qspi for external flash memory
    result = cy_serial_flash_qspi_init(smifMemConfigs[MEM_SLOT_NUM], CYBSP_QSPI_D0, CYBSP_QSPI_D1, CYBSP_QSPI_D2, CYBSP_QSPI_D3, NC, NC, NC, NC, CYBSP_QSPI_SCK, CYBSP_QSPI_SS, QSPI_BUS_FREQUENCY_HZ);
	if(result != CY_RSLT_SUCCESS){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "INIT FAILED\r\n");
		CY_ASSERT(0);
	}

	// Write buffer to beginning of external flash memory - need to erase first
	result = cy_serial_flash_qspi_erase(startOfFlash, cy_serial_flash_qspi_get_erase_size(startOfFlash));
	if(result != CY_RSLT_SUCCESS){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "ERASE FAILED\r\n");
		CY_ASSERT(0);
	}
    result = cy_serial_flash_qspi_write(startOfFlash, MESSAGE_LENGTH, (const uint8_t *)buffer);
    if(result != CY_RSLT_SUCCESS){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "WRITE FAILED\r\n");
		CY_ASSERT(0);
    }

	// Enable Execute in Place (memory mapped) mode - this allows external flash to be read by DMA
	result = cy_serial_flash_qspi_enable_xip(true);
	if(result != CY_RSLT_SUCCESS){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "XIP MODE FAILED\r\n");
		CY_ASSERT(0);
	}

	/////////////////////////////Timer Config////////////////////////////////////////////
	Cy_TCPWM_Counter_Init(MY_TIMER_HW, MY_TIMER_NUM, &MY_TIMER_config);
	Cy_TCPWM_Counter_Enable(MY_TIMER_HW, MY_TIMER_NUM);
	Cy_TCPWM_TriggerStart(MY_TIMER_HW, MY_TIMER_MASK);

	/////////////////////////////DMA Config////////////////////////////////////////////

	// Status variable for DMA initialization
	cy_en_dma_status_t dma_init_status;

	/* DMA interrupt initialization structure */
	cy_stc_sysint_t DMA_INT_cfg =
	{
		.intrSrc      = (IRQn_Type)MY_DMA_IRQ,
		.intrPriority = DMA_INT_PRIORITY,
	};

	/* Initialize and enable the DMA interrupt */
	Cy_SysInt_Init(&DMA_INT_cfg, &Isr_DMA);
	NVIC_EnableIRQ(DMA_INT_cfg.intrSrc);

	/* Enable interrupt for DMA channel */
	Cy_DMA_Channel_SetInterruptMask(MY_DMA_HW, MY_DMA_CHANNEL, CY_DMA_INTR_MASK);

	/* Initialize descriptor 0 */
	dma_init_status = Cy_DMA_Descriptor_Init(&MY_DMA_Descriptor_0, &MY_DMA_Descriptor_0_config);
	if(dma_init_status != CY_DMA_SUCCESS){
		CY_ASSERT(0);
	}

	/* Set source and destination for descriptor 0 */
	Cy_DMA_Descriptor_SetSrcAddress(&MY_DMA_Descriptor_0, (void *) startOfFlashGlobal);
	Cy_DMA_Descriptor_SetDstAddress(&MY_DMA_Descriptor_0, (void *) buffer_r);

	/* Initialize DMA channel */
	dma_init_status = Cy_DMA_Channel_Init(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_channelConfig);
	CY_ASSERT(dma_init_status == CY_DMA_SUCCESS);

	/* Set Descriptor for DMA Channel */
	Cy_DMA_Channel_SetDescriptor(MY_DMA_HW, MY_DMA_CHANNEL, &MY_DMA_Descriptor_0);

	/* Enable DMA block */
	Cy_DMA_Enable(MY_DMA_HW);

	/* Enable Dma channel */
	Cy_DMA_Channel_Enable(MY_DMA_HW, MY_DMA_CHANNEL);

	/////////////////////////////////////////////////////////////////////////

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "ModusToolbox Level 2 - PSoC - Read from external flash using DMA\r\n");
	Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");

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
	if(!(CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus(MY_DMA_HW, MY_DMA_CHANNEL))){
		Cy_SCB_UART_PutString(DEBUG_UART_HW, "DMA Error Occurred. Halting Execution.\r\n");
		CY_ASSERT(0);
	}

	Cy_SCB_UART_PutArray(DEBUG_UART_HW, buffer_r, MESSAGE_LENGTH);

	/* Clear Dma channel interrupt */
	Cy_DMA_Channel_ClearInterrupt(MY_DMA_HW, MY_DMA_CHANNEL);
}
/* [] END OF FILE */
