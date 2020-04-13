/*-
 * $Copyright$
-*/

#ifndef _FRAME_FILTER_HPP_3532115b_b86b_48e5_a4c1_810606f79dfe
#define _FRAME_FILTER_HPP_3532115b_b86b_48e5_a4c1_810606f79dfe

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

template<typename InputBufferT, typename OutputBufferT, size_t N, typename FilterChainT, typename PinT, typename UartT>
class FrameFilterT : public Task {
private:
    UartT &                 m_uart;
    OutputBufferT           (&m_frames)[N];
    FilterChainT &          m_filter;
    PinT *                  m_activeIndicationPin;
    unsigned                m_current;
    QueueHandle_t *         m_rxQueue;
    QueueHandle_t *         m_txQueue;

    virtual void run(void);

public:
    FrameFilterT(const char * const p_name, const unsigned p_priority,
      UartT &p_uart, OutputBufferT (&p_frames)[N], FilterChainT &p_filter);
    virtual ~FrameFilterT();

    void setTxQueue(QueueHandle_t* p_txQueue) {
        m_txQueue = p_txQueue;
    }

    QueueHandle_t* getTxQueue() const {
        return m_txQueue;
    }

    void setRxQueue(QueueHandle_t* p_rxQueue) {
        m_rxQueue = p_rxQueue;
    }

    QueueHandle_t* getRxQueue() const {
        return m_rxQueue;
    }

    void setActiveIndicationPin(PinT* p_activeIndicationPin) {
        m_activeIndicationPin = p_activeIndicationPin;
    }

    PinT* getActiveIndicationPin() const {
        return m_activeIndicationPin;
    }

    typedef OutputBufferT Buffer_t;
};

}; /* namespace tasks */

#include "FrameFilter.cpp"

#endif /* _FRAME_FILTER_HPP_3532115b_b86b_48e5_a4c1_810606f79dfe */