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
#include "cybsp.h"
#include "bmi160.h"
#include "stdio.h"

#define I2C_WRITE_BUFFER_LENGTH   32

// I2C context variable
cy_stc_scb_i2c_context_t I2C_context;

/*
 * I2C ISR
 */
void I2C_Isr(void){
    Cy_SCB_I2C_MasterInterrupt(I2C_HW, &I2C_context);
}

/*
 * Motion sensor I2C write function
 */
static int8_t i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
	// Create write buffer
    CY_ASSERT(len + 1 < I2C_WRITE_BUFFER_LENGTH);
    uint8_t buf[I2C_WRITE_BUFFER_LENGTH];
    buf[0] = reg_addr;
    for(uint16_t i=0; i<len; i++){
        buf[i+1] = data[i];
    }

    // I2C transfer configuration structure
    cy_stc_scb_i2c_master_xfer_config_t xferConfig = {
		.slaveAddress = dev_addr,
		.buffer = buf,
		.bufferSize = len + 1,
		.xferPending = false
	};

    // Write
    cy_en_scb_i2c_status_t result = Cy_SCB_I2C_MasterWrite(I2C_HW, &xferConfig, &I2C_context);
    // Wait for write to complete
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context))){}

	// Return the result of the write
    return (CY_SCB_I2C_SUCCESS == result) ? BMI160_OK : BMI160_E_COM_FAIL;
}

/*
 * Motion sensor I2C read function
 */
static int8_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
	// I2C transfer configuration structure
	cy_stc_scb_i2c_master_xfer_config_t xferConfig = {
		.slaveAddress = dev_addr,
		.buffer = &reg_addr,
		.bufferSize = 1,
		.xferPending = true
	};

	// Write
	cy_en_scb_i2c_status_t result = Cy_SCB_I2C_MasterWrite(I2C_HW, &xferConfig, &I2C_context);
	// Wait for write to complete
	while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context))){}

	// If write was successful we can now read
    if (CY_SCB_I2C_SUCCESS == result){
    	// Reconfigure transfer configuration structure for read
		xferConfig.buffer = data;
		xferConfig.bufferSize = len;
		xferConfig.xferPending = false;

		// Read
		result = Cy_SCB_I2C_MasterRead(I2C_HW, &xferConfig, &I2C_context);
		// Wait for read to complete
		while (0UL != (CY_SCB_I2C_MASTER_BUSY & Cy_SCB_I2C_MasterGetStatus(I2C_HW, &I2C_context))){}
    }

    // Return the result of the read
    return (CY_SCB_I2C_SUCCESS == result) ? BMI160_OK : BMI160_E_COM_FAIL;
}

/*
 * Motion sensor delay function
 */
static void delay_wrapper(uint32_t ms){
    (void)Cy_SysLib_Delay(ms);
}

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

    // UART context variable
	cy_stc_scb_uart_context_t UART_context;

    // Configure and enable the UART peripheral
	Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
	Cy_SCB_UART_Enable(UART_HW);

	// Initialize and enable the I2C peripheral
	Cy_SCB_I2C_Init(I2C_HW, &I2C_config, &I2C_context);

	// I2C interrupt config - Interrupt is needed to use the PDL's I2C Master High-level functions
	const cy_stc_sysint_t i2cIntrConfig =
	{
		.intrSrc      = I2C_IRQ,
		.intrPriority = 7UL,
	};
	Cy_SysInt_Init(&i2cIntrConfig, &I2C_Isr);
	NVIC_EnableIRQ(I2C_IRQ);
	Cy_SCB_I2C_Enable(I2C_HW);

	// Motion sensor device configuration struct
    struct bmi160_dev motionSensor = {
		.id = BMI160_I2C_ADDR,
		.interface = BMI160_I2C_INTF,
		.read         = (bmi160_com_fptr_t)i2c_read_bytes,
		.write        = (bmi160_com_fptr_t)i2c_write_bytes,
		.delay_ms     = delay_wrapper

    };

    // Intialize and configure the motion sensor
    int8_t status = bmi160_init(&motionSensor);
    if(status != BMI160_OK){
    	Cy_SCB_UART_PutString(UART_HW, "Motion Sensor Initialization Failed!\n");
    	CY_ASSERT(0);
    }
    // Select the Output data rate, range of accelerometer sensor
	motionSensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
	motionSensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
	motionSensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

	// Select the power mode of accelerometer sensor
	motionSensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

	// Select the Output data rate, range of gyroscope sensor
	motionSensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
	motionSensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
	motionSensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

	// Gyroscope power mode
	motionSensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

	// Set the sensor configuration
	status = bmi160_set_sens_conf(&motionSensor);
	if(status != BMI160_OK){
		Cy_SCB_UART_PutString(UART_HW, "Motion Sensor Configuration Failed!\n");
		CY_ASSERT(0);
	}

	// Vars to hold motion data
	struct bmi160_sensor_data accel; // Accelerometer data
	struct bmi160_sensor_data gyro; // Gyroscope data

    for (;;){
    	// Read data from the motion sensor and print it to the Debug UART every 100ms
    	status = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &motionSensor);
    	if(status != BMI160_OK){
			Cy_SCB_UART_PutString(UART_HW, "Motion Sensor Read Failed!\n");
			CY_ASSERT(0);
		}
    	char printBuffer[50];
    	sprintf(printBuffer, "Accel: X:%6d Y:%6d Z:%6d\r\n", accel.x, accel.y, accel.z);
    	Cy_SCB_UART_PutString(UART_HW, printBuffer);
		sprintf(printBuffer, "Gyro : X:%6d Y:%6d Z:%6d\r\n\r\n", gyro.x, gyro.y, gyro.z);
		Cy_SCB_UART_PutString(UART_HW, printBuffer);
		Cy_SysLib_Delay(100);
    }
}

/* [] END OF FILE */
