#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"

#define ULP_WAKEUP_INTERVAL_MS 1000

//SCL = 2 or 4
#define SCL_PIN GPIO_NUM_4
//SDA = 0 or 15
#define SDA_PIN GPIO_NUM_15

#define SLAVE1_ADDR 0x0
#define SLAVE1_SUBADDR 0x0

// #define SLAVE2_ADDR 0x0
// #define SLAVE2_SUBADDR 0x0

RTC_DATA_ATTR ulp_var_t ulp_data1;
RTC_DATA_ATTR ulp_var_t ulp_data2;

void init_ulp()
{
    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

        I_I2C_READ(0, SLAVE1_SUBADDR),
        I_PUT(R0,R2,ulp_data1),

#ifdef SLAVE2_ADDR
        I_I2C_READ(1, SLAVE2_SUBADDR),
        I_PUT(R0,R2,ulp_data2),
#endif

        I_HALT(),
    };

    
    hulp_register_i2c_slave(0, SLAVE1_ADDR);
#ifdef SLAVE2_ADDR
    hulp_register_i2c_slave(1, SLAVE2_ADDR);
#endif

    ESP_ERROR_CHECK(hulp_configure_i2c_pins(SCL_PIN, SDA_PIN, true, true));

    const hulp_i2c_controller_config_t config = HULP_I2C_CONTROLLER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(hulp_configure_i2c_controller(&config));
    
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), ULP_WAKEUP_INTERVAL_MS * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

extern "C" void app_main()
{
    init_ulp();

    for(;;)
    {
        vTaskDelay(ULP_WAKEUP_INTERVAL_MS / portTICK_PERIOD_MS);
        printf("Slave 1 Data: %u\n", ulp_data1.val);
        printf("Slave 2 Data: %u\n", ulp_data2.val);
    }
}