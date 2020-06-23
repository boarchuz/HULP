#ifndef HULP_ULP_H
#define HULP_ULP_H

#include "esp32/ulp.h"

#include "sdkconfig.h"

#ifndef ULP_RESERVE_MEM
#ifdef CONFIG_ESP32_ULP_COPROC_RESERVE_MEM
#define ULP_RESERVE_MEM CONFIG_ESP32_ULP_COPROC_RESERVE_MEM
#else
#define ULP_RESERVE_MEM CONFIG_ESP32S2_ULP_COPROC_RESERVE_MEM
#endif
#endif

#endif // HULP_ULP_H