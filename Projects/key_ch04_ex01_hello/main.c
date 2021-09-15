/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 6 MCU: Hello World Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
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


/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_lptimer_event_t event);


/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;

/* Timer object used for blinking the LED */
cyhal_lptimer_t led_blink_timer;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It sets up a timer to trigger a 
* periodic interrupt. The main while loop checks for the status of a flag set 
* by the interrupt and toggles an LED at 1Hz to create an LED blinky. The 
* while loop also checks whether the 'Enter' key was pressed and 
* stops/restarts LED blinking.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the User LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, 
                             CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* GPIO init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }


    /* Initialize timer to toggle the LED */
    timer_init();
 
    for(;;)
	{
		cyhal_syspm_deepsleep();

		if(timer_interrupt_flag)
		{
			timer_interrupt_flag = false;

			if (led_blink_active_flag)
			{
				cyhal_gpio_toggle((cyhal_gpio_t) CYBSP_USER_LED);
			}
		}
	}

}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks 
* continuously and produces a periodic interrupt on every terminal count 
* event. The period is defined by the 'period' and 'compare_value' of the 
* timer configuration structure 'led_blink_timer_cfg'. Without any changes, 
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
#define LPTIMER_MATCH_VALUE (32767)
#define LPTIMER_INTR_PRIORITY (3u)

void timer_init(void)
{
   /* Initialize lptimer. */
   cyhal_lptimer_init(&led_blink_timer );

   /* CLK_LF is 32,768 Hz, so 32,767 counts give us a 1 second interrupt */
   cyhal_lptimer_set_match(&led_blink_timer, LPTIMER_MATCH_VALUE);

   /* Register the interrupt callback handler */
   cyhal_lptimer_register_callback(&led_blink_timer, isr_timer, NULL);

   /* Configure and Enable the LPTIMER events */
   cyhal_lptimer_enable_event(&led_blink_timer,
             CYHAL_LPTIMER_COMPARE_MATCH, LPTIMER_INTR_PRIORITY, true);

   /* Reload/Reset the Low-Power timer to get periodic interrupt. */
   cyhal_lptimer_reload(&led_blink_timer);
}



/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_lptimer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main loop */
    timer_interrupt_flag = true;

    /* Reload/Reset the LPTIMER to get periodic interrupt */
    cyhal_lptimer_reload(&led_blink_timer);
}


/* [] END OF FILE */
