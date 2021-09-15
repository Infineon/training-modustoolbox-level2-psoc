/******************************************************************************
* File Name: main.c
*
* Description: This code example features a 5-segment CapSense slider and two
*              CapSense buttons. Button 0 turns the LED ON, Button 1 turns the
*              LED OFF and the slider controls the brightness of the LED. The
*              code example also features interfacing with Tuner GUI using I2C
*              interface.
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
 * Include header files
 ******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "led.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define CAPSENSE_INTR_PRIORITY      (7u)
#define EZI2C_INTR_PRIORITY         (6u) /* EZI2C interrupt priority must be
                                          * higher than CapSense interrupt */

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t initialize_capsense(void);
static void process_touch(void);
static void initialize_capsense_tuner(void);
static void capsense_isr(void);
static void capsense_callback();
volatile void handle_error(void);

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;
cyhal_ezi2c_t sEzI2C;
cyhal_ezi2c_slave_cfg_t sEzI2C_sub_cfg;
cyhal_ezi2c_cfg_t sEzI2C_cfg;
volatile bool capsense_scan_complete = false;

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

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

    initialize_led();
    initialize_capsense_tuner();
    result = initialize_capsense();

    if (CYRET_SUCCESS != result)
    {
        /* Halt the CPU if CapSense initialization failed */
        CY_ASSERT(0);
    }

    /* Initiate first scan */
    Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

    for (;;)
    {
        if (capsense_scan_complete)
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Initiate next scan */
            Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

            capsense_scan_complete = false;

            cyhal_syspm_sleep(); /* Sleep until CapSense completes a scan */
        }

    }
    
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button0_status;
    uint32_t button1_status;
    cy_stc_capsense_touch_t *slider_touch_info;
    uint16_t slider_pos;
    uint8_t slider_touch_status;
    bool led_update_req = false;

    static uint32_t button0_status_prev;
    static uint32_t button1_status_prev;
    static uint16_t slider_pos_prev;
    static led_data_t led_data = {LED_ON, LED_MAX_BRIGHTNESS};

    /* Get button 0 status */
    button0_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON0_WDGT_ID,
        CY_CAPSENSE_BUTTON0_SNS0_ID,
        &cy_capsense_context);

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(
        CY_CAPSENSE_BUTTON1_WDGT_ID,
        CY_CAPSENSE_BUTTON1_SNS0_ID,
        &cy_capsense_context);

    /* Get slider status */
    slider_touch_info = Cy_CapSense_GetTouchInfo(
        CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
    slider_touch_status = slider_touch_info->numPosition;
    slider_pos = slider_touch_info->ptrPosition->x;

    /* Detect new touch on Button0 */
    if ((0u != button0_status) &&
        (0u == button0_status_prev))
    {
        led_data.state = LED_ON;
        led_update_req = true;
    }

    /* Detect new touch on Button1 */
    if ((0u != button1_status) &&
        (0u == button1_status_prev))
    {
        led_data.state = LED_OFF;
        led_update_req = true;
    }

    /* Detect the new touch on slider */
    if ((0 != slider_touch_status) &&
        (slider_pos != slider_pos_prev))
    {
        led_data.brightness = (slider_pos * 100)
                / cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].xResolution;
        led_update_req = true;
    }

    /* Update the LED state if requested */
    if (led_update_req)
    {
        update_led_state(&led_data);
    }

    /* Update previous touch status */
    button0_status_prev = button0_status;
    button1_status_prev = button1_status;
    slider_pos_prev = slider_pos;
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configure the CapSense
*  interrupt.
*
*******************************************************************************/
static uint32_t initialize_capsense(void)
{
    uint32_t status = CYRET_SUCCESS;

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t CapSense_interrupt_config =
        {
            .intrSrc = CYBSP_CSD_IRQ,
            .intrPriority = CAPSENSE_INTR_PRIORITY,
        };

    /* Capture the CSD HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Initialize CapSense interrupt */
    Cy_SysInt_Init(&CapSense_interrupt_config, capsense_isr);
    NVIC_ClearPendingIRQ(CapSense_interrupt_config.intrSrc);
    NVIC_EnableIRQ(CapSense_interrupt_config.intrSrc);

    /* Initialize the CapSense firmware modules. */
    status = Cy_CapSense_Enable(&cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    /* Assign a callback function to indicate end of CapSense scan. */
    status = Cy_CapSense_RegisterCallback(CY_CAPSENSE_END_OF_SCAN_E,
            capsense_callback, &cy_capsense_context);
    if (CYRET_SUCCESS != status)
    {
        return status;
    }

    return status;
}

/*******************************************************************************
* Function Name: capsense_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense block.
*
*******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: capsense_callback()
********************************************************************************
* Summary:
*  This function sets a flag to indicate end of a CapSense scan.
*
* Parameters:
*  cy_stc_active_scan_sns_t* : pointer to active sensor details.
*
*******************************************************************************/
void capsense_callback(cy_stc_active_scan_sns_t * ptrActiveScan)
{
    capsense_scan_complete = true;
}


/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
*  Initializes interface between Tuner GUI and PSoC 6 MCU.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_rslt_t result;

    /* Configure Capsense Tuner as EzI2C Slave */
    sEzI2C_sub_cfg.buf = (uint8 *)&cy_capsense_tuner;
    sEzI2C_sub_cfg.buf_rw_boundary = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.buf_size = sizeof(cy_capsense_tuner);
    sEzI2C_sub_cfg.slave_address = 8U;

    sEzI2C_cfg.data_rate = CYHAL_EZI2C_DATA_RATE_400KHZ;
    sEzI2C_cfg.enable_wake_from_sleep = false;
    sEzI2C_cfg.slave1_cfg = sEzI2C_sub_cfg;
    sEzI2C_cfg.sub_address_size = CYHAL_EZI2C_SUB_ADDR16_BITS;
    sEzI2C_cfg.two_addresses = false;
    
    result = cyhal_ezi2c_init(&sEzI2C, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL, &sEzI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

}

/* [] END OF FILE */
