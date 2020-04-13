/*-
 * $Copyright$
-*/

#ifndef _TASKS_LED_MATRIX_DISPLAY_HPP_98fcd3b4_59e6_4abe_826a_08fe4d10dd22
#define _TASKS_LED_MATRIX_DISPLAY_HPP_98fcd3b4_59e6_4abe_826a_08fe4d10dd22

#include "tasks/Heartbeat.hpp"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename LedMatrixT, typename UartT, unsigned t_width, unsigned t_height>
LedMatrixDisplayT<LedMatrixT, UartT, t_width, t_height>::LedMatrixDisplayT(const char * const p_name, UartT &p_uart, LedMatrixT &p_ledMatrix,
 const unsigned p_priority, const unsigned p_periodUs /* = 500 */)
  : Task(p_name, p_priority), m_uart(p_uart), m_ledMatrix(p_ledMatrix), m_periodUs(p_periodUs) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename LedMatrixT, typename UartT, unsigned t_width, unsigned t_height>
LedMatrixDisplayT<LedMatrixT, UartT, t_width, t_height>::~LedMatrixDisplayT() {
}

/*******************************************************************************
 *
 ******************************************************************************/
static uint8_t red[] = {
    0xFF, 0xFF, 0xC3, 0xC3,
    0xC3, 0xC3, 0xFF, 0xFF,
};

static uint8_t green[] = {
    0xC3, 0xC3, 0xFF, 0xFF,
    0xFF, 0xFF, 0xC3, 0xC3
};

static uint8_t blue[] = {
    0x18, 0x18, 0x18, 0x18,
    0x18, 0x18, 0x18, 0x18,
};

template<typename LedMatrixT, typename UartT, unsigned t_width, unsigned t_height>
void
LedMatrixDisplayT<LedMatrixT, UartT, t_width, t_height>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);

    const TickType_t period = this->m_periodUs / portTICK_RATE_MICROSECONDS;

    for (unsigned column = 0; column < t_width; column++) {
        this->m_ledMatrix.setColumn(column, LedMatrixT::e_RgbChannel_Red, red[column]);
        this->m_ledMatrix.setColumn(column, LedMatrixT::e_RgbChannel_Green, green[column]);
        this->m_ledMatrix.setColumn(column, LedMatrixT::e_RgbChannel_Blue, blue[column]);
    }

    while (1) {
        for (unsigned column = 0; column < t_width ; column++) {
            for (unsigned channel = 0; channel < 3; channel++) {
                this->m_ledMatrix.refresh(column, static_cast<typename LedMatrixT::RgbChannel_t>(channel));
                vTaskDelay(period);
            }
        };
    }
}

} /* namespace tasks */

#endif /* _TASKS_LED_MATRIX_DISPLAY_HPP_98fcd3b4_59e6_4abe_826a_08fe4d10dd22 */
