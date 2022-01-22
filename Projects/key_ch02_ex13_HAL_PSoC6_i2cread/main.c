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
#include "mtb_bmi160.h"

#define IMU_I2C_SDA (P6_1)
#define IMU_I2C_SCL (P6_0)

mtb_bmi160_t motion_sensor;
cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize i2c for motion sensor */
    result = cyhal_i2c_init(&i2c, IMU_I2C_SDA, IMU_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
        result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize motion sensor */
    result = mtb_bmi160_init_i2c(&motion_sensor, &i2c, MTB_BMI160_DEFAULT_ADDRESS);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    
    for (;;)
    {
        /* Get the accel and gyro data and print the results to the UART */
        mtb_bmi160_data_t data;
        mtb_bmi160_read(&motion_sensor, &data);

        printf("Accel: X:%6d Y:%6d Z:%6d\r\n", data.accel.x, data.accel.y, data.accel.z);
        printf("Gyro : X:%6d Y:%6d Z:%6d\r\n\r\n", data.gyro.x, data.gyro.y, data.gyro.z);

        cyhal_system_delay_ms(100);
    }
}

/* [] END OF FILE */
