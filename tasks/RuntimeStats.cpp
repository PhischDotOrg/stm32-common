/*-
 * $Copyright$
-*/

#ifndef _RUNTIME_STATS_CPP_e8059c84_38ef_4d64_878a_929d2e1b6ae1
#define _RUNTIME_STATS_CPP_e8059c84_38ef_4d64_878a_929d2e1b6ae1

#include <tasks/RuntimeStats.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {
    
/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT>
RuntimeStatsT<UartT>::RuntimeStatsT(const char* const p_name, const unsigned p_priority,
  UartT& p_uart, const unsigned p_periodInMs) : Task(p_name, p_priority), m_uart(p_uart), m_periodInMs(p_periodInMs) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT>
RuntimeStatsT<UartT>::~RuntimeStatsT(void) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT>
void
RuntimeStatsT<UartT>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);
    const TickType_t period = (this->m_periodInMs * 1000) / portTICK_RATE_MICROSECONDS;
    TickType_t lastWakeTime = xTaskGetTickCount();
    unsigned count = 0;
    while(1) {
        vTaskDelayUntil(&lastWakeTime, period);
        vTaskGetRunTimeStats(this->m_buffer);
        this->m_uart.printf("%s: Count=%u\r\n", this->m_name, count);
        this->m_uart.printf("%s\r\n", this->m_buffer);
        count++;
    }

    this->m_uart.printf("%s() ended!\r\n", this->m_name);
    halt(__FILE__, __LINE__);
}

} /* namespace tasks */

#endif /* _RUNTIME_STATS_CPP_e8059c84_38ef_4d64_878a_929d2e1b6ae1 */