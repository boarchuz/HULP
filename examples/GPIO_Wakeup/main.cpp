/* ULP GPIO Wakeup Example

   This uses the deep sleep wakeup stub to add GPIO wakeup functionality to ESP32 ULP.

   Upon powering on, the ULP program is loaded, an EXT1 wakeup trigger is configured, a custom deep 
   sleep wakeup stub is set, and deep sleep begins.

   In a typical application, the EXT1 trigger GPIO might be an interrupt signal from an I2C slave, and the ULP
   might read some data, apply filtering, and determine whether or not to wake the SoC.
   In this example, GPIO 0 is used as the wakeup trigger as it is usually connected to a button on most dev boards, 
   and the ULP simply increments a counter, waking the SoC when it reaches 5.

   For optimal power consumption, the stub strives to return to sleep asap. It disables EXT1, starts ULP, and immediately 
   sleeps; the ULP handles processing, waits until the button is released, and then re-enables the EXT1 interrupt in 
   preparation for the next trigger.

   Note that the ULP sleep timer is used here even though the ULP disables it anyway at the end of the program. This is 
   because triggering a single ULP run in deep sleep proved difficult: it would often use the 150kHz clock (instead of
   8MHz); and RTC memory would power down once the stub ended (often while the ULP is running) causing lockup. Using the
   timer forces the RTC controller to handle all of this which is much easier and more reliable.
   
   In practice, ULP runs ~1ms after trigger edge. When idle, expect nominal ESP32 deep sleep current (5-10uA).
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp32/rom/rtc.h"
#include "soc/uart_reg.h"
#include "soc/timer_group_reg.h"

#include "hulp.h"

//Uncomment for UART info and/or to scope STUB_DEBUG_GPIO and ULP_DEBUG_GPIO
// #define DEBUG_MODE

#define TRIGGER_GPIO GPIO_NUM_0

RTC_DATA_ATTR ulp_var_t ulp_counter;
RTC_DATA_ATTR ulp_var_t ulp_wake_signal;

#ifdef DEBUG_MODE
#define STUB_DEBUG_GPIO GPIO_NUM_26
#define ULP_DEBUG_GPIO  GPIO_NUM_27
RTC_FAST_ATTR int stub_rtc_io;
RTC_RODATA_ATTR static const char debug_fmt_str[] = "ULP Counter: %u, Wake Flag: %u\n";

static inline void RTC_IRAM_ATTR stub_uart_flush()
{
    while (REG_GET_FIELD(UART_STATUS_REG(0), UART_ST_UTX_OUT)) {
        REG_WRITE(TIMG_WDTFEED_REG(0), 1);
        ;
    }
}

static inline void RTC_IRAM_ATTR stub_set_rtcio(uint32_t rtcio, uint32_t level)
{
    if(level)
    {
        REG_SET_BIT(RTC_GPIO_OUT_W1TS_REG, BIT(RTC_GPIO_OUT_DATA_W1TS_S + rtcio));
    }
    else
    {
        REG_SET_BIT(RTC_GPIO_OUT_W1TC_REG, BIT(RTC_GPIO_OUT_DATA_W1TC_S + rtcio));
    }
}
#endif // DEBUG_MODE

static inline void RTC_IRAM_ATTR stub_start_ulp()
{
    REG_SET_BIT(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}

static inline void RTC_IRAM_ATTR stub_set(esp_deep_sleep_wake_stub_fn_t stub)
{
    REG_WRITE(RTC_ENTRY_ADDR_REG, (uint32_t)stub);
}

static inline void RTC_IRAM_ATTR stub_return_to_sleep()
{
    // See https://gist.github.com/igrr/54f7fbe0513ac14e1aea3fd7fbecfeab
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
    SET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_SLEEP_EN);
    // A few CPU cycles may be necessary for the sleep to start...
    for(;;);
}

static inline void RTC_IRAM_ATTR stub_disable_ext1()
{
    REG_CLR_BIT(RTC_CNTL_WAKEUP_STATE_REG, BIT(RTC_CNTL_WAKEUP_ENA_S + 1));
}

static void RTC_IRAM_ATTR wake_stub()
{
#ifdef DEBUG_MODE
    stub_set_rtcio(stub_rtc_io, 0);
    stub_set_rtcio(stub_rtc_io, 1);
    ets_printf(debug_fmt_str, ulp_counter.val, ulp_wake_signal.val);
    stub_uart_flush();
#endif

    stub_disable_ext1();

    //If the ULP has set ulp_wake_signal then do full wakeup
    if(ulp_wake_signal.val != 0)
    {
        ulp_wake_signal.val = 0;
        esp_default_wake_deep_sleep();
        return;
    }

    //Else start ULP and go back to sleep
    stub_start_ulp();

    stub_set(&wake_stub);

    stub_return_to_sleep();
}

void prepare_ulp()
{
    enum {
        LBL_BELOW_THRESHOLD,
        LBL_FINISH_UP,
    };
    
    const ulp_insn_t program[] = {
#ifdef DEBUG_MODE
        //Toggle GPIO
        I_GPIO_SET(ULP_DEBUG_GPIO, 0),
        I_DELAY(65535),
        I_GPIO_SET(ULP_DEBUG_GPIO, 1),
#endif

        //Increment counter
        I_MOVI(R2, 0),
        I_GET(R0, R2, ulp_counter),
        I_ADDI(R0, R0, 1),
        I_PUT(R0, R2, ulp_counter),

        //Check if counter has reached threshold
        M_BL(LBL_BELOW_THRESHOLD, 5),
        
        //Set ulp_wake_signal
        I_MOVI(R1, 1),
        I_PUT(R1, R2, ulp_wake_signal),
        //Use the EXT1 interrupt to wake the SoC again by driving the pin low
        //And loop until stub resets the flag to 0 to acknowledge
        I_GPIO_OUTPUT_EN(TRIGGER_GPIO),
        I_EXT1_EN(),
        I_GET(R0, R2, ulp_wake_signal),
        I_BGE(-3, 1),
        //Stop driving the pin low so button can interrupt again
        I_GPIO_OUTPUT_DIS(TRIGGER_GPIO),
        //Reset counter
        I_PUT(R2, R2, ulp_counter),
        M_BX(LBL_FINISH_UP),

        M_LABEL(LBL_BELOW_THRESHOLD),
            //Wait for button to be released
            I_GPIO_READ(TRIGGER_GPIO),
            I_BL(-1, 1),
            //Debounce a few ms
            I_DELAY(65535),
            
        M_LABEL(LBL_FINISH_UP),
            //Re-enable EXT1 so next button press will run wakeup stub
            I_EXT1_EN(),
            //Disable timer (ie. only run once)
            I_END(),
            I_HALT(),
                
    };
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 0, 0));

    //Prepare ULP, but do not enable timer so it won't run yet (see ulp_run())
    REG_SET_FIELD(SENS_SAR_START_FORCE_REG, SENS_PC_INIT, 0);
    CLEAR_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_FORCE_START_TOP);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_I2C_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_CORE_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_SLEEP_FOLW_8M);
}

extern "C" void app_main(void)
{
    if(hulp_is_deep_sleep_wakeup())
    {
        printf("Woken up!\n");
    }
    else
    {
#ifdef DEBUG_MODE
        stub_rtc_io = rtc_io_number_get(STUB_DEBUG_GPIO);
        assert(stub_rtc_io != -1);
        hulp_configure_pin(STUB_DEBUG_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_PULLUP_ONLY, 1);
        hulp_configure_pin(ULP_DEBUG_GPIO, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_PULLUP_ONLY, 1);
#endif
        hulp_configure_pin(TRIGGER_GPIO, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0);
        prepare_ulp();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    hulp_peripherals_on();
    esp_set_deep_sleep_wake_stub(&wake_stub);
    ESP_ERROR_CHECK( esp_sleep_enable_ext1_wakeup(1ULL << TRIGGER_GPIO, ESP_EXT1_WAKEUP_ALL_LOW) );
    esp_deep_sleep_start();
}
