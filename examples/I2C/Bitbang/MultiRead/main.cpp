#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"
#include "hulp_i2cbb.h"

#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13

#define SLAVE1_ADDR 0x12
#define SLAVE1_SUBADDR 0x0
#define SLAVE1_INTERVAL_MS 1000

#define SLAVE2_ADDR 0x34
#define SLAVE2_SUBADDR 0x0
#define SLAVE2_INTERVAL_MS 2500

RTC_DATA_ATTR ulp_var_t ulp_data1;
RTC_DATA_ATTR ulp_var_t ulp_data2;
RTC_DATA_ATTR ulp_var_t ulp_nacks;
RTC_DATA_ATTR ulp_var_t ulp_buserrors;

void startulp()
{
    enum {
        LBL_SLAVE1_INTERVAL,
        LBL_SLAVE1_RETURN,

        LBL_SLAVE2_INTERVAL,
        LBL_SLAVE2_RETURN,

        LBL_HALT,
        
        LBL_I2C_READ_ENTRY,
        LBL_I2C_WRITE_ENTRY,
        LBL_I2C_NACK,
        LBL_I2C_ARBLOST,
    };

    const ulp_insn_t program[]{
        I_MOVI(R2,0),

        M_UPDATE_TICKS(),

        M_IF_MS_ELAPSED(LBL_SLAVE1_INTERVAL, SLAVE1_INTERVAL_MS, LBL_SLAVE2_INTERVAL),
            I_I2CBB_SET_SLAVE(SLAVE1_ADDR),
            M_I2CBB_RD(LBL_SLAVE1_RETURN, LBL_I2C_READ_ENTRY, SLAVE1_SUBADDR),
            I_PUT(R0, R2, ulp_data1),

        M_IF_MS_ELAPSED(LBL_SLAVE2_INTERVAL, SLAVE2_INTERVAL_MS, LBL_HALT),
            I_I2CBB_SET_SLAVE(SLAVE2_ADDR),
            M_I2CBB_RD(LBL_SLAVE2_RETURN, LBL_I2C_READ_ENTRY, SLAVE2_SUBADDR),
            I_PUT(R0, R2, ulp_data2),

        M_LABEL(LBL_HALT),
            I_HALT(),

        M_LABEL(LBL_I2C_NACK),
            I_GET(R0, R2, ulp_nacks),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_nacks),
            I_BXR(R3),

        M_LABEL(LBL_I2C_ARBLOST),
            I_GET(R0, R2, ulp_buserrors),
            I_ADDI(R0,R0,1),
            I_PUT(R0,R2, ulp_buserrors),
            I_BXR(R3),

        M_INCLUDE_I2CBB_MULTI(LBL_I2C_READ_ENTRY, LBL_I2C_WRITE_ENTRY, LBL_I2C_ARBLOST, LBL_I2C_NACK, SCL_PIN, SDA_PIN),
    };

    hulp_configure_pin(SCL_PIN,RTC_GPIO_MODE_INPUT_ONLY,false,false,0);
    hulp_configure_pin(SDA_PIN,RTC_GPIO_MODE_INPUT_ONLY,false,false,0);

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    hulp_ulp_load(program, sizeof(program), 1ULL * 50 * 1000);
    hulp_ulp_run();
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
        if(ulp_nacks.updated())
        {
            ulp_nacks.clearUpdated();
            printf("NACKs: %u\n", ulp_nacks.get());
        }
        if(ulp_buserrors.updated())
        {
            ulp_buserrors.clearUpdated();
            printf("Bus errors: %u\n", ulp_buserrors.get());
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}