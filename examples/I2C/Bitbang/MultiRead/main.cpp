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

void init_ulp()
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

    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

        M_UPDATE_TICKS(),

        M_IF_MS_ELAPSED(LBL_SLAVE1_INTERVAL, SLAVE1_INTERVAL_MS, LBL_SLAVE2_INTERVAL),
            I_I2CBB_SET_SLAVE(SLAVE1_ADDR),
            M_I2CBB_RD(LBL_SLAVE1_RETURN, LBL_I2C_READ_ENTRY, SLAVE1_SUBADDR),
            I_PUT(R0, R2, ulp_data1),
            I_WAKE(),

        M_IF_MS_ELAPSED(LBL_SLAVE2_INTERVAL, SLAVE2_INTERVAL_MS, LBL_HALT),
            I_I2CBB_SET_SLAVE(SLAVE2_ADDR),
            M_I2CBB_RD(LBL_SLAVE2_RETURN, LBL_I2C_READ_ENTRY, SLAVE2_SUBADDR),
            I_PUT(R0, R2, ulp_data2),
            I_WAKE(),

        M_LABEL(LBL_HALT),
            I_HALT(),

        M_LABEL(LBL_I2C_NACK),
            I_GET(R0, R2, ulp_nacks),
            I_ADDI(R0, R0, 1),
            I_PUT(R0, R2, ulp_nacks),
            I_WAKE(),
            I_BXR(R3),

        M_LABEL(LBL_I2C_ARBLOST),
            I_GET(R0, R2, ulp_buserrors),
            I_ADDI(R0, R0, 1),
            I_PUT(R0, R2, ulp_buserrors),
            I_WAKE(),
            I_BXR(R3),

        M_INCLUDE_I2CBB_MULTI(LBL_I2C_READ_ENTRY, LBL_I2C_WRITE_ENTRY, LBL_I2C_ARBLOST, LBL_I2C_NACK, SCL_PIN, SDA_PIN),
    };

    hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0);
    hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0);

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    hulp_ulp_load(program, sizeof(program), 1ULL * 50 * 1000);
    hulp_ulp_run();
}

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

extern "C" void app_main()
{
    //ULP will trigger an interrupt when there's new data or an error
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, &main_handle);
    hulp_ulp_interrupt_en();

    init_ulp();

    for(;;)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY); //Wait for interrupt
        printf("Data1: %u, Data2: %u, NACK Errors: %u, Bus Errors: %u", ulp_data1.val, ulp_data2.val, ulp_nacks.val, ulp_buserrors.val);
    }
}