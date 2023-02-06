#include "cyhal.h"
#include "cybsp.h"
#include "mtb_ssd1306.h"
#include "GUI.h"
#include "xensiv_dps3xx_mtb.h"

xensiv_dps3xx_t pressure_sensor;

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

    /* Initialize i2c */
    result = cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_i2c_configure(&i2c, &i2c_cfg);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize OLED display */
    result = mtb_ssd1306_init_i2c(&i2c);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, XENSIV_DPS3XX_I2C_ADDR_DEFAULT);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    GUI_Init();
    GUI_DispString("Pressure:");
    GUI_DispStringAt("Temperature:", 0, 10);

    for(;;)
    {
        /* Get the pressure and temperature data and print the results to the UART */
         float pressure, temperature;

         xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);

         GUI_GotoXY(75, 0);
         GUI_DispFloatMin(pressure,2);

         GUI_GotoXY(75, 10);
         GUI_DispFloatMin(temperature,2);

         cyhal_system_delay_ms(500);
    }
}
