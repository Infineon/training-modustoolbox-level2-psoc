#include "cyhal.h"
#include "cybsp.h"
#include "mtb_ssd1306.h"
#include "GUI.h"

int main(void)
{
    cy_rslt_t result;
    cyhal_i2c_t i2c_obj;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

     if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the I2C to use with the OLED display */
    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize OLED display */
    result = mtb_ssd1306_init_i2c(&i2c_obj);

    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    GUI_Init();
    GUI_DispString("Hello World!");

    for(;;)
    {
    }
}
