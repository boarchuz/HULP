
#ifndef HULP_UART_H
#define HULP_UART_H

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"

#include "hulp.h"

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HULP_UART_STRING_INIT_SIZE(capacity) {{.val = (capacity) << 8}}

/**
 * Helper to declare an array of ulp_var_t to be used as a buffer with the given capacity
 *  eg. RTC_SLOW_ATTR ulp_var_t my_buffer HULP_UART_STRING_BUFFER(8); // Can hold string of max length 8
 */
#define HULP_UART_STRING_BUFFER(required_capacity) [(1 + ((required_capacity) / 2))] = HULP_UART_STRING_INIT_SIZE(required_capacity)

/**
 * Helper to declare an array of ulp_var_t of sufficient size to hold the given string literal.
 * hulp_uart_string_set must be used to then populate the array.
 *  eg. 
 *      #define MY_ULP_STRING "Hello world"
 *      RTC_SLOW_ATTR ulp_var_t my_ulp_string HULP_UART_STRING_RESERVE(MY_ULP_STRING); // Large enough to hold MY_ULP_STRING
 *      hulp_uart_string_set(my_ulp_string, sizeof(my_ulp_string) / sizeof(ulp_var_t), MY_ULP_STRING); // Set MY_ULP_STRING
 */
#define HULP_UART_STRING_RESERVE(target_str) [(1 + (sizeof(target_str) / 2))] = HULP_UART_STRING_INIT_SIZE(sizeof(target_str))

/**
 * Set the array of ulp_var_t to the provided string, formatted for usage by the ULP.
 * len: Array length of ulp_var_t, ie. (sizeof(my_string) / sizeof(ulp_var_t))
 * 
 * Returns -1 if any errors
 */
int hulp_uart_string_set(ulp_var_t *hulp_string, size_t len, const char* str);

/**
 * Get the string from the array of ulp_var_t into the provided buffer.
 * 
 * Returns -1 if any errors
 */
int hulp_uart_string_get(ulp_var_t *hulp_string, char* buffer, size_t buffer_size, bool clear);

/**
 * ULP subroutine to receive data over UART, until receiving the provided termination byte or the buffer is full.
 * 
 * Prep:
 * Set R1 = offset of ULP string eg. I_MOVO(R1, ulp_receive_buffer)
 * Put return address in R3.     eg. M_MOVL(R3, LABEL_RETURN_POINT)
 * Branch to label_entry         eg. M_BX(LABEL_UART_RX)
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
        I_DELAY((uint16_t)(hulp_get_fast_clk_freq() / 2 / (baud_rate) + 34 - 36)),                                                   \
        I_DELAY((uint16_t)(hulp_get_fast_clk_freq() / (baud_rate) - 34)),                                                        \
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

/**
 * ULP subroutine to transmit data over UART.
 * 
 * Prep:
 * Set R1 = offset of ULP string (eg. I_MOVO(R1, ulp_receive_buffer),)
 * Put return address in R3.
 * Branch to label_entry
 */
#define M_INCLUDE_UART_TX(label_entry, baud_rate, tx_gpio) \
    M_INCLUDE_UART_TX_(label_entry, baud_rate, tx_gpio, R1, R2, R3)

#if CONFIG_HULP_UART_TX_OD
#define I_HULP_UART_TX_HIGH(pin) I_GPIO_OUTPUT_DIS(pin)
#define I_HULP_UART_TX_LOW(pin) I_GPIO_OUTPUT_EN(pin)
#else
#define I_HULP_UART_TX_HIGH(pin) I_GPIO_SET(pin, 1)
#define I_HULP_UART_TX_LOW(pin) I_GPIO_SET(pin, 0)
#endif

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
        I_HULP_UART_TX_LOW((tx_gpio)),\
        I_DELAY((uint16_t)(hulp_get_fast_clk_freq() / (baud_rate) - 19)), /* Start bit, minus a little overhead */ \
        I_ANDI(R0,reg_scr,1),\
        I_BL(12, 1),\
        I_HULP_UART_TX_HIGH((tx_gpio)),\
        I_DELAY((uint16_t)((hulp_get_fast_clk_freq() / (baud_rate)) - 42)),\
        I_RSHI(reg_scr,reg_scr,1),\
        I_STAGE_INC(1),\
        I_JUMPS(2, 9, JUMPS_LT),\
        I_JUMPS(-7, 16, JUMPS_LT),\
        I_JUMPS(-8, 8, JUMPS_LT),\
        I_HULP_UART_TX_HIGH((tx_gpio)),\
        I_DELAY((uint16_t)(hulp_get_fast_clk_freq() / (baud_rate) - 0)),\
        I_JUMPS(-16, 9, JUMPS_LT),\
        I_BGE(-20, 0),\
        I_HULP_UART_TX_LOW((tx_gpio)),\
        I_DELAY((uint16_t)((hulp_get_fast_clk_freq() / (baud_rate)) - 48)),\
        I_BGE(-11, 0),\
        M_MOVL(reg_return,label_entry),\
        I_LD(reg_return,reg_return,31),\
        I_BXR(reg_return),\
        I_HALT()

/**
 * This will convert the value in R0 into an ASCII string (decimal). Very useful for debugging in combination with UART TX.
 * 
 *  As R1, R2, and R3 are also used in the subroutine, you should store the value beforehand and retrieve afterwards if still required.
 *  The string is always 5 digits, padded with leading zeros where necessary (ie. "00000", "00001", ... "65535").
 *  A ulp_string_t initialised with a size of 6 is required for the destination buffer. eg. RTC_DATA_ATTR ulp_string_t<6> ulp_printf_buffer;
 * 
 *  A newline character will be appended automatically. The ULP may therefore pass the buffer directly to UART TX to print one value per line.
 *  Use the customisable variant to override this behaviour.
 *  
 *  Prepare R1 with the offset of the ulp_string_t buffer. eg. I_MOVO(R1, ulp_printf_buffer)
 *  Prepare R3 with the return address. eg. MOVL(R3, LABEL_SOME_PRINTF_RETURN_POINT)
 *  Branch to the entry of the subroutine.
 */
#define M_INCLUDE_PRINTF_U(label_entry) \
    M_INCLUDE_PRINTF_U_(label_entry, R1, R2, R3, 1, '\n')

/**
 * Given that ulp_string_t rounds up odd initialisation sizes, a length of 5 for our 5-digit value produces a string with max length of 6 (one unused char).
 * May as well take advantage of it (eg. using '\n' allows outputting the buffer directly to UART TX with a linebreak between values, zero overhead).
 * If desired, use_final_char == 1 and specify final_char. To output the 5 digit ASCII only, use_final_char == 0.
 */
#define M_INCLUDE_PRINTF_U_(label_entry, reg_string_ptr, reg_scr, reg_return, use_final_char, final_char) \
    M_LABEL(label_entry), \
        I_ST(R0, reg_string_ptr, 3), \
        I_MOVI(reg_scr, '0' << 8 | '0'),  /* Reset counters for digits (10^4) (lower bits) and (10^3) (upper bits) with ASCII zero '0' */ \
        I_BL(4, 10000),              /* Loop incrementing (10^4) counter: '0' + 1 = '1' + 1 = '2' + 1 = '3' etc */ \
        I_ADDI(reg_scr, reg_scr, 1),  \
        I_SUBI(R0, R0, 10000),  \
        I_BGE(-3, 0),   \
        I_BL(4, 1000),              /* Loop incrementing (10^3) counter */ \
        I_ADDI(reg_scr, reg_scr, 1 << 8), \
        I_SUBI(R0, R0, 1000),   \
        I_BGE(-3, 0),   \
        I_ST(reg_scr, reg_string_ptr, 1),            /* Store ASCII counters for digits (10^4) and (10^3) at offset metadata+1 */ \
        I_MOVI(reg_scr, '0' << 8 | '0'), /* Do it all over again for (10^2) and (10^1), at metadata+2 */ \
        I_BL(4, 100),   \
        I_ADDI(reg_scr, reg_scr, 1),  \
        I_SUBI(R0, R0, 100),    \
        I_BGE(-3, 0),   \
        I_BL(4, 10),    \
        I_ADDI(reg_scr, reg_scr, 1 << 8), \
        I_SUBI(R0, R0, 10), \
        I_BGE(-3, 0),   \
        I_ST(reg_scr, reg_string_ptr, 2),    \
        I_MOVI(reg_scr, ((use_final_char) ? (uint8_t)(final_char) : 0) << 8 | '0'),    /* Lastly the units (10^0) at metadata+3 */    \
        I_ADDR(reg_scr, R0, reg_scr),  \
        I_LD(R0, reg_string_ptr, 3), \
        I_ST(reg_scr, reg_string_ptr, 3),    \
        I_LD(reg_scr, reg_string_ptr, 0),    /* Load the metadata (ie. offset+0) and set length=5*/    \
        I_ANDI(reg_scr, reg_scr, 0xFF << 8),  \
        I_ORI(reg_scr, reg_scr, 5 + ((use_final_char) ? 1 : 0)),   \
        I_ST(reg_scr, reg_string_ptr, 0),    \
        I_BXR(reg_return)

/**
 * Format value as hex string. See M_INCLUDE_PRINTF_U
 */
#define M_INCLUDE_PRINTF_X(label_entry) \
    M_INCLUDE_PRINTF_X_(label_entry, R1, R2, R3)

#define M_INCLUDE_PRINTF_X_(label_entry, reg_string_ptr, reg_scr, reg_return) \
    M_LABEL(label_entry), \
        I_ST(R0, reg_string_ptr, 2), \
        I_MOVI(reg_scr, '0' << 8 | '0'), \
        I_BL(3, 40960),   \
        I_ADDI(reg_scr, reg_scr, ('A' - '0')),  \
        I_SUBI(R0, R0, 40960),    \
        I_BL(4, 4096),   \
        I_ADDI(reg_scr, reg_scr, 1),  \
        I_SUBI(R0, R0, 4096),    \
        I_BGE(-3, 0),   \
        I_BL(3, 2560),    \
        I_ADDI(reg_scr, reg_scr, ('A' - '0') << 8), \
        I_SUBI(R0, R0, 2560), \
        I_BL(4, 256),    \
        I_ADDI(reg_scr, reg_scr, 1 << 8), \
        I_SUBI(R0, R0, 256), \
        I_BGE(-3, 0),   \
        I_ST(reg_scr, reg_string_ptr, 1),    \
        I_MOVI(reg_scr, '0' << 8 | '0'), \
        I_BL(3, 160),   \
        I_ADDI(reg_scr, reg_scr, 'A' - '0'),  \
        I_SUBI(R0, R0, 160),    \
        I_BL(4, 16),   \
        I_ADDI(reg_scr, reg_scr, 1),  \
        I_SUBI(R0, R0, 16),    \
        I_BGE(-3, 0),   \
        I_BL(3, 10),    \
        I_ADDI(reg_scr, reg_scr, ('A' - '0') << 8), \
        I_SUBI(R0, R0, 10), \
        I_LSHI(R0, R0, 8), \
        I_ADDR(reg_scr, R0, reg_scr), \
        I_LD(R0, reg_string_ptr, 2), \
        I_ST(reg_scr, reg_string_ptr, 2),    \
        I_LD(reg_scr, reg_string_ptr, 0),    /* Load the metadata (ie. offset+0) and set length=4*/    \
        I_ANDI(reg_scr, reg_scr, 0xFF << 8),  \
        I_ORI(reg_scr, reg_scr, 4),   \
        I_ST(reg_scr, reg_string_ptr, 0),    \
        I_BXR(reg_return)

#ifdef __cplusplus
}
#endif

#endif // HULP_UART_H