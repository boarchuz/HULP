#ifndef HULP_MUTEX_H
#define HULP_MUTEX_H

/**
 * Mutex implementation based on Peterson's Algorithm, for managing access to resources shared between ULP and SoC
 */

#include "hulp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mutex object for managing resource access between ULP and SoC
 *
 * ULP:
 *  Take with M_MUTEX_TAKE (or M_MUTEX_TRY_TAKE), release with M_MUTEX_RELEASE
 * SoC:
 *  Take with HULP_MUTEX_SOC_TAKE_WHILE, release with HULP_MUTEX_SOC_RELEASE
 */
typedef struct {
    ulp_var_t ulp_request;
    ulp_var_t soc_request;
    ulp_var_t soc_priority;
} ulp_mutex_t;

/**
 * ULP: Flag mutex wanted
 *
 * mutex: ulp_mutex_t object
 * reg_nonzero: R0-R3 which has any known value != 0
 * reg_nonzero_val: The known value in specified register
 */
#define M_MUTEX_REQUEST(_mutex, _reg_nonzero, _reg_nonzero_val) \
    I_PUTO(_reg_nonzero, _reg_nonzero, _reg_nonzero_val, (_mutex).ulp_request), \
    I_PUTO(_reg_nonzero, _reg_nonzero, _reg_nonzero_val, (_mutex).soc_priority)

/**
 * ULP: Check if mutex held (after M_MUTEX_REQUEST)
 * R0 == 0 on success
 *
 * mutex: ulp_mutex_t object
 * reg_any: R1-R3 which has any known value
 * reg_any_val: The known value in specified register
 */
#define M_MUTEX_CHECK(_mutex, _reg_any, _reg_any_val) \
    I_GETO(R0, _reg_any, _reg_any_val, (_mutex).soc_request), \
    I_BL(2, 1), \
    I_GETO(R0, _reg_any, _reg_any_val, (_mutex).soc_priority)

/**
 * ULP: Take mutex (non-blocking)
 * R0 == 0 on success
 * Must release (M_MUTEX_RELEASE) regardless of result
 *
 * mutex: ulp_mutex_t object
 * reg_nonzero: R1-R3 which has any known value != 0
 * reg_nonzero_val: The known value in specified register
 */
#define M_MUTEX_TRY_TAKE(_mutex, _reg_nonzero, _reg_nonzero_val) \
    M_MUTEX_REQUEST(_mutex, _reg_nonzero, _reg_nonzero_val), \
    M_MUTEX_CHECK(_mutex, _reg_nonzero, _reg_nonzero_val)

/**
 * ULP: Take mutex
 *
 * mutex: ulp_mutex_t object
 * reg_nonzero: R1-R3 which has any known value != 0
 * reg_nonzero_val: The known value in specified register
 */
#define M_MUTEX_TAKE(_mutex, _reg_nonzero, _reg_nonzero_val) \
    M_MUTEX_TRY_TAKE(_mutex, _reg_nonzero, _reg_nonzero_val), \
    I_BGE(-3, 1)

/**
 * ULP: Release mutex
 *
 * mutex: ulp_mutex_t object previously taken
 * reg_zero: R0-R3 which has known value of 0
 */
#define M_MUTEX_RELEASE(_mutex, _reg_zero) \
    I_PUT(_reg_zero, _reg_zero, (_mutex).ulp_request)

/**
 * SoC: Flag mutex wanted
 */
static inline void hulp_mutex_soc_request(volatile ulp_mutex_t *mutex)
{
    mutex->soc_request.val = 1;
    mutex->soc_priority.val = 0;
}

/**
 * SoC: Check if mutex held (after hulp_mutex_soc_request)
 */
static inline bool hulp_mutex_soc_check(const volatile ulp_mutex_t *mutex)
{
    return mutex->ulp_request.val == 0 || mutex->soc_priority.val != 0;
}

/**
 * SoC: Release mutex
 */
static inline void hulp_mutex_soc_release(volatile ulp_mutex_t *mutex)
{
    mutex->soc_request.val = 0;
}

/**
 * SoC: Take mutex
 *
 * mutex: pointer to ulp_mutex_t object
 *
 * eg.
 *  HULP_MUTEX_SOC_TAKE_WHILE(&ulp_led_mutex) {
 *      // Do this while waiting for ULP to release
 *      vTaskDelay(1); // or ets_delay_us(10), etc
 *  }
 *  // Critical Section
 *  // ...
 *  HULP_MUTEX_SOC_RELEASE(&ulp_led_mutex);
 */
#define HULP_MUTEX_SOC_TAKE_WHILE(_mutex) \
    hulp_mutex_soc_request(_mutex); \
    while(!hulp_mutex_soc_check(_mutex))

#define HULP_MUTEX_SOC_RELEASE hulp_mutex_soc_release

#ifdef __cplusplus
}
#endif

#endif /* HULP_MUTEX_H */
