#ifndef PRESSURE_CONFIG_H
#define PRESSURE_CONFIG_H

#include "app_config.h"

// Derived pressure switches (hierarchy: enable -> source -> channel).
#define APP_PRESSURE_ENABLE        (FEATURE_PRESSURE_ENABLE)
#define APP_PRESSURE_SOURCE_WF     (APP_PRESSURE_ENABLE && (FEATURE_PRESSURE_SOURCE == 1))
#define APP_PRESSURE_SOURCE_DC     (APP_PRESSURE_ENABLE && (FEATURE_PRESSURE_SOURCE == 0))
#define APP_PRESSURE_DC_CH1        (APP_PRESSURE_SOURCE_DC && FEATURE_PRESSURE_DC_CH1)
#define APP_PRESSURE_DC_CH2        (APP_PRESSURE_SOURCE_DC && FEATURE_PRESSURE_DC_CH2)

#if APP_PRESSURE_ENABLE
    #if (FEATURE_PRESSURE_SOURCE != 0) && (FEATURE_PRESSURE_SOURCE != 1)
        #error "FEATURE_PRESSURE_SOURCE must be 0 (DC) or 1 (WF)"
    #endif
    #if APP_PRESSURE_SOURCE_WF && !FEATURE_WF5803F_ENABLE
        #error "WF pressure requires FEATURE_WF5803F_ENABLE=1"
    #endif
    #if APP_PRESSURE_SOURCE_DC
        #if (FEATURE_PRESSURE_DC_CH1 == 0) && (FEATURE_PRESSURE_DC_CH2 == 0)
            #error "Select at least one DC pressure channel: CH1 or CH2"
        #endif
    #endif
#else
    #undef APP_PRESSURE_SOURCE_WF
    #define APP_PRESSURE_SOURCE_WF 0
    #undef APP_PRESSURE_SOURCE_DC
    #define APP_PRESSURE_SOURCE_DC 0
#endif

#if APP_PRESSURE_SOURCE_DC
    #if APP_PRESSURE_DC_CH1
        #define APP_PRESSURE_DC_ADC_CMD APP_EXT_ADC_CMD_Press1
    #else
        #define APP_PRESSURE_DC_ADC_CMD APP_EXT_ADC_CMD_Press2
    #endif
#endif

#endif  // PRESSURE_CONFIG_H
