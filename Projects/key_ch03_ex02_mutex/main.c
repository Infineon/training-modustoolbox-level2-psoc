#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

volatile int uxTopUsedPriority;

//TODO create variable for mutex
static SemaphoreHandle_t LED_lock;

/* LED task 1 - blink at 2 Hz */
void led_task1()
{
    while (1)
    {
        //TODO lock the mutex
    	xSemaphoreTake(LED_lock, portMAX_DELAY);

        cyhal_gpio_toggle(CYBSP_USER_LED);

        //TODO unlock the mutex
    	xSemaphoreGive(LED_lock);

        vTaskDelay (1000/2); /* 2 Hz delay and give the other task a turn */
    }
}

/* LED task 2 - blink at 5 Hz when button is pressed */
void led_task2()
{
    while (1)
    {

        //TODO lock the mutex
    	xSemaphoreTake(LED_lock, portMAX_DELAY);

        while (CYBSP_BTN_PRESSED == cyhal_gpio_read(CYBSP_USER_BTN)) /* Button is pressed */
        {
        	cyhal_gpio_toggle(CYBSP_USER_LED);
        	vTaskDelay (1000/5);
        }

        //TODO unlock the mutex
        xSemaphoreGive(LED_lock);

        vTaskDelay (1);	/* Give the other task a turn when button is not pressed */
    }
}

int main(void)
{
    uxTopUsedPriority = configMAX_PRIORITIES - 1;

    /* Initialize the device and board peripherals */
	cybsp_init() ;
    __enable_irq();

    /* Initialize the User LED */
    cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize User Button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);

	//TODO Create Mutex
    LED_lock = xSemaphoreCreateMutex();

    /* Create tasks with a stack of 1024, no parameters, and a priority of 1 */
    xTaskCreate(led_task1, (char *)"led_task1", 1024, 0, 1, NULL);
    xTaskCreate(led_task2, (char *)"led_task2", 1024, 0, 1, NULL);

    /* Start the RTOS scheduler */
    vTaskStartScheduler();
    CY_ASSERT(0);
}

/* [] END OF FILE */
