/*-
 * $Copyright$
-*/

#ifndef _FRAME_DISPLAYER_HPP_51c8c1a0_a2ac_4631_be6e_7bf46178980c
#define _FRAME_DISPLAYER_HPP_51c8c1a0_a2ac_4631_be6e_7bf46178980c

#include <tasks/Task.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/queue.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {

template<typename InputBufferT, typename DisplayT, typename UartT>
class FrameDisplayerT : public Task {
private:
    UartT &                 m_uart;
    DisplayT &              m_display;
    QueueHandle_t *         m_rxQueue;

    virtual void run(void);

public:
    FrameDisplayerT(const char * const p_name, const unsigned p_priority,
      UartT &p_uart, DisplayT &p_display);
    virtual ~FrameDisplayerT();

    void setRxQueue(QueueHandle_t* p_rxQueue) {
        m_rxQueue = p_rxQueue;
    }

    QueueHandle_t* getRxQueue() const {
        return m_rxQueue;
    }
};

}; /* namespace tasks */

#include "FrameDisplayer.cpp"

#endif /* _FRAME_DISPLAYER_HPP_51c8c1a0_a2ac_4631_be6e_7bf46178980c */
