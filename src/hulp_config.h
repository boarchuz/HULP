#ifndef HULP_CONFIG_H
#define HULP_CONFIG_H

#include "sdkconfig.h"

// Arduino build does not generate sdkconfig. Set missing config items to default values.
#ifndef CONFIG_HULP_LABEL_AUTO_BASE
    #define CONFIG_HULP_LABEL_AUTO_BASE 60000
#endif
#ifndef CONFIG_HULP_FAST_CLK_CAL_CYCLES
    #define CONFIG_HULP_FAST_CLK_CAL_CYCLES 100
#endif

#endif // HULP_CONFIG_H