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

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy8ckit_028_tft.h"
#include "GUI.h"

int main(void){

    /* Light and Motion sensor objects (the objects are setup by the shield init function) */
    mtb_light_sensor_t* lightObj;
    mtb_bmi160_t* motionObj;

    /* Light and Motion sensor data */
    uint8_t lightData;
	mtb_bmi160_data_t motionData;

    cy_rslt_t result;
    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
        // Enable interrupts
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* Initialize the CY8CKIT_028_TFT board */
    /* This will initialize the TFT, Motion Sensor, Light Sensor and I2C for the motion sensor */
    result = cy8ckit_028_tft_init (NULL, NULL, NULL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    /* Initialize the emwin library */
	GUI_Init();

    /* Get objects for the light sensor data (from ADC) and motion sensor data (from I2C) */
    /* that were set up during the shield initialization */
    lightObj = cy8ckit_028_tft_get_light_sensor();
    motionObj = cy8ckit_028_tft_get_motion_sensor();

	//print headers
	GUI_DispString("Light Level: ");
	GUI_DispStringAt("Accel: X:       Y:       Z:", 0, 10);
	GUI_DispStringAt("Gyro:  X:       Y:       Z:", 0, 20);

    for(;;){
        lightData = mtb_light_sensor_light_level(lightObj); //Get light level
    	mtb_bmi160_read(motionObj, &motionData); //Get Motion data

    	//print data
        GUI_DispDecAt(lightData, 75, 0, 3);
        GUI_DispDecAt(motionData.accel.x, 55, 10, 6);
        GUI_DispDecAt(motionData.accel.y, 110, 10, 6);
        GUI_DispDecAt(motionData.accel.z, 165, 10, 6);
        GUI_DispDecAt(motionData.gyro.x, 55, 20, 6);
        GUI_DispDecAt(motionData.gyro.y, 110, 20, 6);
        GUI_DispDecAt(motionData.gyro.z, 165, 20, 6);
        //delay half a second
        cyhal_system_delay_ms(500);
    }
}

/* [] END OF FILE */
