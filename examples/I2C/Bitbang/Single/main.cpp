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

#define SLAVE_ADDR 0x0

// Set for 8-bit read:
// #define SLAVE_READ8_SUBADDR 0x0

// Set for 16-bit read:
// #define SLAVE_READ16_SUBADDR 0x0

// Set subaddress and value for write:
// #define SLAVE_WRITE_SUBADDR 0x0
// #define SLAVE_WRITE_VALUE 0x0

RTC_DATA_ATTR ulp_var_t ulp_data8;
RTC_DATA_ATTR ulp_var_t ulp_data16;
RTC_DATA_ATTR ulp_var_t ulp_nacks;
RTC_DATA_ATTR ulp_var_t ulp_buserrors;

void startulp()
{
    enum {
        LBL_READ8_RETURN,
        LBL_READ16_RETURN,
        LBL_WRITE_RETURN,

        LBL_HALT,
        
        LBL_I2C_READ_ENTRY,
        LBL_I2C_WRITE_ENTRY,
        LBL_I2C_NACK,
        LBL_I2C_ARBLOST,
    };

    const ulp_insn_t program[]{
        I_MOVI(R2,0),

    #ifdef SLAVE_READ8_SUBADDR
        M_I2CBB_RD(LBL_READ8_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ8_SUBADDR),
        I_PUT(R0, R2, ulp_data8),
    #endif

    #ifdef SLAVE_READ16_SUBADDR
        M_I2CBB_RD(LBL_READ16_RETURN, LBL_I2C_READ_ENTRY, SLAVE_READ16_SUBADDR),
        I_PUT(R0, R2, ulp_data16),
    #endif

    #ifdef SLAVE_WRITE_SUBADDR
        M_I2CBB_WR(LBL_WRITE_RETURN, LBL_I2C_WRITE_ENTRY, SLAVE_WRITE_SUBADDR, SLAVE_WRITE_VALUE),
    #endif

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

        M_INCLUDE_I2CBB(LBL_I2C_READ_ENTRY, LBL_I2C_WRITE_ENTRY, LBL_I2C_ARBLOST, LBL_I2C_NACK, SCL_PIN, SDA_PIN, SLAVE_ADDR),
    };

    hulp_configure_pin(SCL_PIN,RTC_GPIO_MODE_INPUT_ONLY,false,false,0);
    hulp_configure_pin(SDA_PIN,RTC_GPIO_MODE_INPUT_ONLY,false,false,0);

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    hulp_start(program, sizeof(program), 1ULL * 1000 * 1000);
}

extern "C" void app_main()
{
    startulp();

    for(;;)
    {
        if(ulp_data8.updated())
        {
            ulp_data8.clearUpdated();
            printf("Read (8): %u\n", ulp_data8.get());
        }
        if(ulp_data16.updated())
        {
            ulp_data16.clearUpdated();
            printf("Read (16): %u\n", ulp_data16.get());
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