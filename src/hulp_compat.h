#ifndef HULP_COMPAT_H
#define HULP_COMPAT_H

#include "esp_idf_version.h"
#include "sdkconfig.h"

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,2,0)
    #error "Unsupported IDF version"
#endif

#if CONFIG_IDF_TARGET_ESP32
    #include "esp32/clk.h"
    #include "esp32/ulp.h"
    #define HULP_ULP_RESERVE_MEM CONFIG_ESP32_ULP_COPROC_RESERVE_MEM
#else
    #error "Unsupported target"
#endif

#endif // HULP_COMPAT_H