/******************************************************************************
* File Name: led.c
*
* Description: This file contains source code that controls LED.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header files includes
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "led.h"


/*******************************************************************************
* Global constants
*******************************************************************************/
#define PWM_LED_FREQ_HZ    (1000000lu)  /* in Hz */
#define GET_DUTY_CYCLE(x)    (100 - x)


/*******************************************************************************
* Global constants
*******************************************************************************/
led_state_t led_state_cur = LED_OFF;
cyhal_pwm_t pwm_led;


/*******************************************************************************
* Function Name: update_led_state
********************************************************************************
* Summary:
*  This function updates the LED state, based on the touch input.
*
* Parameter:
*  ledData: the pointer to the LED data structure
*
*******************************************************************************/
void update_led_state(led_data_t *ledData)
{
    if ((led_state_cur == LED_OFF) && (ledData->state == LED_ON))
    {
        cyhal_pwm_start(&pwm_led);
        led_state_cur = LED_ON;
        ledData->brightness = LED_MAX_BRIGHTNESS;
    }
    else if ((led_state_cur == LED_ON) && (ledData->state == LED_OFF))
    {
        cyhal_pwm_stop(&pwm_led);
        led_state_cur = LED_OFF;
        ledData->brightness = 0;
    }
    else
    {
    }

    if ((LED_ON == led_state_cur) || ((LED_OFF == led_state_cur) && (ledData->brightness > 0)))
    {
        cyhal_pwm_start(&pwm_led);
        uint32_t brightness = (ledData->brightness < LED_MIN_BRIGHTNESS) ? LED_MIN_BRIGHTNESS : ledData->brightness;

        /* Drive the LED with brightness */
        cyhal_pwm_set_duty_cycle(&pwm_led, GET_DUTY_CYCLE(brightness),
                                 PWM_LED_FREQ_HZ);
        led_state_cur = LED_ON;
    }
}


/*******************************************************************************
* Function Name: initialize_led
********************************************************************************
* Summary:
*  Initializes a PWM resource for driving an LED.
*
*******************************************************************************/
cy_rslt_t initialize_led(void)
{
    cy_rslt_t rslt;

    rslt = cyhal_pwm_init(&pwm_led, CYBSP_USER_LED, NULL);

    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_pwm_set_duty_cycle(&pwm_led,
                                        GET_DUTY_CYCLE(LED_MAX_BRIGHTNESS),
                                        PWM_LED_FREQ_HZ);
        if (CY_RSLT_SUCCESS == rslt)
        {
            rslt = cyhal_pwm_start(&pwm_led);
        }
        
    }

    if (CY_RSLT_SUCCESS == rslt)
    {
        led_state_cur = LED_ON;
    }

    return rslt;
}


/* [] END OF FILE */
