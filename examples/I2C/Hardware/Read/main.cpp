#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"

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

void startulp()
{
    const ulp_insn_t program[]{
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

    hulp_configure_i2c_pins(SCL_PIN, SDA_PIN);
    hulp_configure_i2c_controller();
    
    hulp_start(program, sizeof(program), 1ULL * 1000 * 1000);
}

extern "C" void app_main()
{
    startulp();

    for(;;)
    {
        if(ulp_data1.updated())
        {
            ulp_data1.clearUpdated();
            printf("Slave 1 Data: %u\n", ulp_data1.get());
        }
        if(ulp_data2.updated())
        {
            ulp_data2.clearUpdated();
            printf("Slave 2 Data: %u\n", ulp_data2.get());
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}