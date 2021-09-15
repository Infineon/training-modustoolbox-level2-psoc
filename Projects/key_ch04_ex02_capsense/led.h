/******************************************************************************
* File Name: led.h
*
* Description: This file is the public interface of led.c source file.
*
* Related Document: See README.md
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

/*******************************************************************************
* Include guard
*******************************************************************************/
#ifndef SOURCE_LED_H_
#define SOURCE_LED_H_

/*******************************************************************************
* Global constants
*******************************************************************************/
/* Allowed duty cycle for maximum brightness */
#define LED_MAX_BRIGHTNESS      (100u)

/* Allowed duty cycle for minimum brightness*/
#define LED_MIN_BRIGHTNESS      (2u)

/*******************************************************************************
* Structures and enumerations
*******************************************************************************/
typedef enum
{
    LED_OFF,
    LED_ON
} led_state_t;

typedef struct
{
    led_state_t state;
    uint32_t brightness;
} led_data_t;

/*******************************************************************************
* Function prototypes
*******************************************************************************/
cy_rslt_t initialize_led(void);
void update_led_state(led_data_t *led_data);

#endif /* SOURCE_LED_H_ */

/* [] END OF FILE */
