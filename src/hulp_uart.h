
#ifndef HULP_UART_H_
#define HULP_UART_H_

#include "hulp.h"
#include "soc/rtc.h"

#ifndef HULP_RTC_FAST_CLK_HZ
    //Accurate:
    #define HULP_RTC_FAST_CLK_HZ (1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_clk_cal(RTC_CAL_8MD256, 100))
    //Approx:
    // #define HULP_RTC_FAST_CLK_HZ RTC_FAST_CLK_FREQ_APPROX
#endif

/*
Prep:
Set R1 = offset of ULP string eg. I_MOVO(R1, ulp_receive_buffer)
Put return address in R3.     eg. M_MOVL(R3, LABEL_RETURN_POINT)
Branch to label_entry         eg. M_BX(LABEL_UART_RX)
*/
#define M_INCLUDE_UART_RX(label_entry, baud_rate, rx_gpio, termination_char) \
    M_INCLUDE_UART_RX_(label_entry, baud_rate, rx_gpio, R1, R2, R3, termination_char)

#define M_INCLUDE_UART_RX_(label_entry, baud_rate, rx_gpio, reg_string_ptr, reg_scr, reg_return, termination_char) \
    M_LABEL(label_entry),                                                                                                           \
        M_MOVL(R0,label_entry),                         /*Need all the registers we can get for this one, so the return address */  \
        I_ST(reg_return,R0,34),                         /*  is saved temporarily*/                                                  \
        I_MOVI(reg_scr,0),                              /*reg_scr is used to count received bytes*/                                 \
        I_LD(R0,reg_string_ptr,0),                      /*Begin loop for each byte: Load the metadata*/                             \
        I_RSHI(R0,R0,8),                                /*  Isolate the buffer size from it*/                                       \
        I_SUBR(R0,R0,reg_scr),                          /*  Then compare buffer size with current length to check if full*/         \
        I_BL(21, 1),                                                                                                                \
        I_GPIO_READ(rx_gpio),                           /*Wait here until pin goes low (start bit)*/                                \
        I_BGE(-1,1),                                                                                                                \
        I_STAGE_RST(),                                                                                                              \
        I_DELAY((uint16_t)(0.5f / (baud_rate) * HULP_RTC_FAST_CLK_HZ + 34 - 36)),                                                   \
        I_DELAY((uint16_t)(1.0f / (baud_rate) * HULP_RTC_FAST_CLK_HZ - 34)),                                                        \
        I_GPIO_READ(rx_gpio),                           /*Read the new bit, make room for it in another reg, and OR it in*/         \
        I_RSHI(reg_return,reg_return,1),                                                                                            \
        I_LSHI(R0,R0,15),                                                                                                           \
        I_ORR(reg_return,reg_return,R0),                                                                                            \
        I_STAGE_INC(1),                                                                                                             \
        I_JUMPS(-6, 8, JUMPS_LT),                       /*Repeat 8 times*/                                                          \
        I_RSHI(R0,reg_scr,1),                           /*Store the byte. ulpstring =one word metadata, then 2 chars in every */    \
        I_ADDR(R0,reg_string_ptr,R0),                   /*  word thereafter, so offset = 1+length/2 (ie. length >> 1, */            \
        I_ST(reg_return,R0,1),                          /*  add that to string ptr, then I_ST with 1 offset) */                     \
        I_GPIO_READ(rx_gpio),                           /*Wait here until pin goes high to sync with stop bit*/                     \
        I_BL(-1,1),                                                                                                                 \
        I_SUBI(R0,reg_return,(termination_char)<<8),    /*Most recent byte is in upper 8 bits, so subtract (termination_char)<<8*/  \
        I_BL(3,1<<8),                                   /*If upper bits are 8b0 then byte matches termination_char so end */        \
        I_ADDI(reg_scr,reg_scr,1),                      /*  else increment length and loop back to beginning of new byte */         \
        I_BGE(-23,0),                                                                                                               \
        I_LD(reg_return,reg_string_ptr,0),              /*Load the metadata (termination char / buffer full branches here)*/        \
        I_ANDI(reg_return,reg_return,0xFF<<8),          /*Update metadata with received length*/                                    \
        I_ORR(reg_return,reg_return,reg_scr),                                                                                       \
        I_ST(reg_return,reg_string_ptr,0),              /*  This I_ST also sets updated flag on metadata var */                     \
        M_MOVL(reg_return,label_entry),                 /*Now need to load the return address saved at the beginning, and return */ \
        I_LD(reg_return,reg_return,34),                                                                                             \
        I_BXR(reg_return),                                                                                                          \
        I_HALT()                                       /*reserved word (control should never reach here)*/

/*
Prep:
Set R1 = offset of ULP string (eg. I_MOVO(R1, ulp_receive_buffer),)
Put return address in R3.
Branch to label_entry
*/
#define M_INCLUDE_UART_TX(label_entry, baud_rate, tx_gpio) \
    M_INCLUDE_UART_TX_(label_entry, baud_rate, tx_gpio, R1, R2, R3)

#define M_INCLUDE_UART_TX_(label_entry, baud_rate, tx_gpio, reg_string_ptr, reg_scr, reg_return) \
    M_LABEL(label_entry), \
        M_MOVL(R0,label_entry),\
        I_ST(reg_return,R0,31),\
        I_LD(reg_return,reg_string_ptr,0),\
        I_ANDI(reg_return,reg_return,255),\
        I_ADDI(reg_string_ptr, reg_string_ptr, 1),\
        I_LD(reg_scr,reg_string_ptr,0),\
        I_STAGE_RST(),\
        I_SUBI(reg_return,reg_return,1),\
        I_MOVR(R0,reg_return),\
        I_BGE(19, 65535),\
        I_GPIO_OUTPUT_EN((tx_gpio)),\
        I_DELAY((uint16_t)(1.0f / (baud_rate) * HULP_RTC_FAST_CLK_HZ - 30)),\
        I_ANDI(R0,reg_scr,1),\
        I_BL(12, 1),\
        I_GPIO_OUTPUT_DIS((tx_gpio)),\
        I_DELAY((uint16_t)(1.0f / (baud_rate) * HULP_RTC_FAST_CLK_HZ - 38)),\
        I_RSHI(reg_scr,reg_scr,1),\
        I_STAGE_INC(1),\
        I_JUMPS(-6, 8, JUMPS_LT),\
        I_JUMPS(2, 9, JUMPS_LT),\
        I_JUMPS(-8, 16, JUMPS_LT),\
        I_GPIO_OUTPUT_DIS((tx_gpio)),\
        I_DELAY((uint16_t)(1.0f / (baud_rate) * HULP_RTC_FAST_CLK_HZ)),\
        I_JUMPS(-16, 9, JUMPS_LT),\
        I_BGE(-20, 0),\
        I_GPIO_OUTPUT_EN((tx_gpio)),\
        I_DELAY((uint16_t)(1.0f / (baud_rate) * HULP_RTC_FAST_CLK_HZ - 38 - 4)),\
        I_BGE(-11, 0),\
        M_MOVL(reg_return,label_entry),\
        I_LD(reg_return,reg_return,31),\
        I_BXR(reg_return),\
        I_HALT()

#endif