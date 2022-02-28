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
#include "cyhal.h"
#include "cybsp.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

volatile int uxTopUsedPriority;

static QueueHandle_t queueHandle;

/* Button ISR */
void button_interrupt_handler(void* handler_arg, cyhal_gpio_event_t event)
{
    static BaseType_t xHigherPriorityTaskWoken;
    static uint32_t count = 0;

    /* Increment Count each time the button is pressed */
    count++;

    /* Send count to the queue */
    xQueueSendFromISR(queueHandle, &count, &xHigherPriorityTaskWoken);

    /* Yield current task if a higher priority task is now unblocked */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Structure for GPIO interrupt */
cyhal_gpio_callback_data_t button_interrupt_data =
{
    .callback     = button_interrupt_handler,
    .callback_arg = NULL
};

/* Task to handle the LED */
void led_task()
{
	uint32_t blinkCount = 1;

    while (1)
    {
        /* Wait for new blink value */
    	xQueueReceive(queueHandle, &blinkCount, portMAX_DELAY);

    	/* Blink LED according to count value */
    	for(int i=0; i<blinkCount; i++)
    	{
    		cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_ON);
    		vTaskDelay(100);
    		cyhal_gpio_write(CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
    		vTaskDelay(100);
    	}

    	vTaskDelay(500); /* Wait 500ms between blink sequences so we can tell them apart */
    }
}

int main(void)
{
    cy_rslt_t result;

    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize button with an interrupt of priority 3 */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
	cyhal_gpio_register_callback(CYBSP_USER_BTN, &button_interrupt_data);
	cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, 3, true);

	/* Create Queue */
	queueHandle = xQueueCreate(10, sizeof(uint32_t));

    /* Create task with a stack of 1024, no parameters, and a priority of 1 */
    xTaskCreate(led_task, (char *)"led_task", 1024, 0, 1, NULL);

    /* Start the RTOS scheduler */
    vTaskStartScheduler();
    CY_ASSERT(0);
}

/* [] END OF FILE */
