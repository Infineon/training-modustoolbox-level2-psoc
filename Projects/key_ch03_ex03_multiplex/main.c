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
#include "cyhal.h"
#include "cybsp.h"
#include "cycfg_capsense.h"
#include "cycfg.h"
#include "cy_csdadc.h"
#include <stdlib.h> // abs()

/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY 7U
#define CSDADC_INTR_PRIORITY 7U
#define PWM_FREQ_HZ 100000

/*******************************************************************************
* Global Variables
*******************************************************************************/
bool scanComplete = false;
bool conversionComplete = false;
cyhal_ezi2c_t ezi2c_object;
cyhal_pwm_t pwm_object_red;
cyhal_pwm_t pwm_object_orange;
cy_stc_csdadc_context_t cy_csdadc_context;
const cy_stc_sysint_t CapSense_interrupt_config = {
		.intrSrc = CYBSP_CSD_IRQ,
		.intrPriority = CAPSENSE_INTR_PRIORITY
};
const cy_stc_sysint_t csdadc_interrut_config = {
		.intrSrc = CYBSP_CSD_IRQ,
		.intrPriority = CSDADC_INTR_PRIORITY
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
cy_rslt_t initTuner(void);
cy_rslt_t initCapSense(void);
cy_rslt_t initCSDADC(void);
static void capsense_isr(void);
static void csdadc_isr(void);
void endScanCallback(cy_stc_active_scan_sns_t * ptrActiveScan);
void processTouch(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - initialize CapSense
*  - initialize tuner communication
*  - scan touch input continuously and update the LED accordingly.
*
* Return:
*  int
*
*******************************************************************************/
int main(void){
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS){
        CY_ASSERT(0);
    }

    __enable_irq();

    // Initialize orange PWM
    result = cyhal_pwm_init(&pwm_object_orange, CYBSP_USER_LED, NULL);
    if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}
    result = cyhal_pwm_set_duty_cycle(&pwm_object_orange, 0, PWM_FREQ_HZ);
    if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}

    // Initialize red PWM
    result = cyhal_pwm_init(&pwm_object_red, CYBSP_USER_LED2, NULL);
	if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}
	result = cyhal_pwm_set_duty_cycle(&pwm_object_red, 0, PWM_FREQ_HZ);
	if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}

    // Initialize capsense
    result = initCapSense();
    if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}

    // Save the CSD touch sensing state
    Cy_CapSense_Save(&cy_capsense_context);

    // Initialize CSDADC
    result = initCSDADC();
	if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}

	// Save the CSDADC state
	Cy_CSDADC_Save(&cy_csdadc_context);

	// Restore the CSD touch sensing state
	Cy_CapSense_Restore(&cy_capsense_context);
	Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);

    // Initialize tuner
    result = initTuner();
	if (result != CY_RSLT_SUCCESS){
		CY_ASSERT(0);
	}

	// Start first scan
	Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;){
    	// Wait for scan to complete
    	if(scanComplete == true){

    		// Process scan data
    		Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

    		// Update tuner
    		Cy_CapSense_RunTuner(&cy_capsense_context);

    		// Update LED according to any touches
    		processTouch();

    		// Start next scan
			Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
			scanComplete = false;
    	}
    }
}

/*******************************************************************************
* Function Name: initCapSense
********************************************************************************
* Summary:
* Initializes the CSD HW block for touch sensing
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
cy_rslt_t initCapSense(void){
	// Result to return
	cy_rslt_t result = CY_RSLT_SUCCESS;
	cy_status status = CY_RET_SUCCESS;

	// Initialize CSD HW block
	status = Cy_CapSense_Init(&cy_capsense_context);
	if(status != CY_RET_SUCCESS){
		result = status;
		return result;
	}

	// Initialize CapSense interrupt
	Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
	NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
	NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

	// Enable CapSense firmware modules
	status = Cy_CapSense_Enable(&cy_capsense_context);
	if(status != CY_RET_SUCCESS){
		result = status;
		return result;
	}

	// Initialize end of scan callback
	status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E, endScanCallback, &cy_capsense_context);
	if(status != CY_RET_SUCCESS){
		result = status;
		return result;
	}

	// Return success if nothing failed
	return result;
}

/*******************************************************************************
* Function Name: initCSDADC
********************************************************************************
* Summary:
* Initializes the CSDADC
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
cy_rslt_t initCSDADC(void){
	// Result to return
	cy_rslt_t result = CY_RSLT_SUCCESS;
	cy_status status = CY_RET_SUCCESS;

	// Initialize CSD HW block
	status = Cy_CSDADC_Init(&CYBSP_CSD_csdadc_config, &cy_csdadc_context);
	if(status != CY_RET_SUCCESS){
		result = status;
		return result;
	}

	// Initialize CSDADC interrupt
	Cy_SysInt_Init(&csdadc_interrut_config, csdadc_isr);
	NVIC_ClearPendingIRQ(csdadc_interrut_config.intrSrc);
	NVIC_EnableIRQ(csdadc_interrut_config.intrSrc);

	// Enable CSDADC firmware modules
	Cy_CSDADC_Enable(&cy_csdadc_context);
	if(status != CY_RET_SUCCESS){
		result = status;
		return result;
	}

	// Return success if nothing failed
	return result;
}

/*******************************************************************************
* Function Name: initTuner
********************************************************************************
* Summary:
* Initializes the CapSense tuner
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
cy_rslt_t initTuner(void){
	// Result to return
	cy_rslt_t result = CY_RSLT_SUCCESS;

	cyhal_ezi2c_slave_cfg_t ezi2c_slave_config;
	cyhal_ezi2c_cfg_t ezi2c_config;

	// Configure Capsense Tuner as EzI2C Slave
	ezi2c_slave_config.buf = (uint8 *)&cy_capsense_tuner;
	ezi2c_slave_config.buf_rw_boundary = sizeof(cy_capsense_tuner);
	ezi2c_slave_config.buf_size = sizeof(cy_capsense_tuner);
	ezi2c_slave_config.slave_address = 8U;

	ezi2c_config.data_rate = CYHAL_EZI2C_DATA_RATE_1MHZ;
	ezi2c_config.enable_wake_from_sleep = false;
	ezi2c_config.slave1_cfg = ezi2c_slave_config;
	ezi2c_config.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
	ezi2c_config.two_addresses = false;

	result = cyhal_ezi2c_init(&ezi2c_object, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &ezi2c_config);

	return result;
}

/*******************************************************************************
* Function Name: processTouch
********************************************************************************
* Summary:
* Check if CapSense buttons were pressed and update the LED state accordingly
*
* Return:
*  cy_rslt_t
*
*******************************************************************************/
void processTouch(void){
	// Vars to hold widget statuses
	uint32_t button0_status;
	uint32_t button1_status;
	cy_stc_capsense_touch_t *slider_touch_info;
	uint16_t slider_pos;
	uint8_t slider_touch_status;
	uint32_t pot_millivolts;
	// Vars to hold previous widget statuses
	static uint32_t button0_status_prev;
	static uint32_t button1_status_prev;
	static uint16_t slider_pos_prev;
	static uint32_t pot_millivolts_prev;

	// ADC channel mask
	uint32_t channel0Mask = 0x01U;

	// Get button0 status
	button0_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON0_WDGT_ID, CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context);

	// Get button1 status
	button1_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_BUTTON1_WDGT_ID, CY_CAPSENSE_BUTTON1_SNS0_ID, &cy_capsense_context);

	// Get slider status
	slider_touch_info = Cy_CapSense_GetTouchInfo(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
	slider_touch_status = slider_touch_info->numPosition;
	slider_pos = slider_touch_info->ptrPosition->x;

	// Save CSD state
	Cy_CapSense_Save(&cy_capsense_context);

	// Restore the CSDADC state
	Cy_CSDADC_Restore(&cy_csdadc_context);
	Cy_SysInt_Init(&csdadc_interrut_config, csdadc_isr);

	// Start a conversion
	Cy_CSDADC_StartConvert(CY_CSDADC_SINGLE_SHOT, channel0Mask, &cy_csdadc_context);

	// Wait for conversion to complete
	while(Cy_CSDADC_IsEndConversion(&cy_csdadc_context) != CY_CSDADC_SUCCESS){}

	// Get ADC result
	pot_millivolts = Cy_CSDADC_GetResultVoltage(0U, &cy_csdadc_context);

	// Save CSDADC state
	Cy_CSDADC_Save(&cy_csdadc_context);

	// Restore the CSD touch sensing state
	Cy_CapSense_Restore(&cy_capsense_context);
	Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);

	// If button0 is pressed turn the LEDs on
	if((button0_status != 0U) && (button0_status != button0_status_prev)){
		cyhal_pwm_start(&pwm_object_red);
		cyhal_pwm_start(&pwm_object_orange);
	}

	// If button1 is pressed turn the LEDs off
	if((button1_status != 0U) && (button1_status != button1_status_prev)){
		cyhal_pwm_stop(&pwm_object_red);
		cyhal_pwm_stop(&pwm_object_orange);
	}

	// If slider position is updated, update LED brightness
	if((slider_touch_status != 0U) && (slider_pos != slider_pos_prev)){
		float dutyCycle = (1.0 - ((float)slider_pos / (float)cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution)) * 100;
		cyhal_pwm_set_duty_cycle(&pwm_object_orange, dutyCycle, PWM_FREQ_HZ);
	}

	// If potentiometer position has changed (by a significant amount), update LED brightness
	if(abs((int)pot_millivolts - (int)pot_millivolts_prev) > 5){
		float dutyCycle = 100.0 - ((float)pot_millivolts * (100.0 / 3300.0));
		cyhal_pwm_set_duty_cycle(&pwm_object_red, dutyCycle, PWM_FREQ_HZ);
	}

	// Update previous state vars
	button0_status_prev = button0_status;
	button1_status_prev = button1_status;
	slider_pos_prev = slider_pos;
	pot_millivolts_prev = pot_millivolts;
}

/*******************************************************************************
* Interrupt Service Routines and Callbacks
*******************************************************************************/
static void capsense_isr(void){
	Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

static void csdadc_isr(void){
	Cy_CSDADC_InterruptHandler(CYBSP_CSD_HW, &cy_csdadc_context);
}

void endScanCallback(cy_stc_active_scan_sns_t * ptrActiveScan){
	scanComplete = true;
}

/* [] END OF FILE */
