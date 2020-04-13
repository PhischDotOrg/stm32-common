/*-
 * $Copyright$
-*/

#ifndef _FRAMESAMPLER_HPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a
#define _FRAMESAMPLER_HPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a

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

template<typename BufferT, size_t N, class TimerChannelT, class AdcT, typename UartT>
class FrameSamplerT : public Task {
private:
    UartT &             m_uart;
    BufferT             (&m_frames)[N];
    TimerChannelT &     m_timerChannel;
    AdcT &              m_adc;
    unsigned            m_current;
    const unsigned      m_periodMs;
    QueueHandle_t *     m_txQueue;

    virtual void run(void);

public:
    FrameSamplerT(const char * const p_name, const unsigned p_priority,
      UartT &p_uart, BufferT (&p_frames)[N], TimerChannelT &p_timerChannel, AdcT &p_adc,
      const unsigned p_periodMs);
    virtual ~FrameSamplerT();

    void setTxQueue(QueueHandle_t* p_txQueue) {
        m_txQueue = p_txQueue;
    }

    QueueHandle_t* getTxQueue() const {
        return m_txQueue;
    }
};

}; /* namespace tasks */

#include "FrameSampler.cpp"

#endif /* _FRAMELOOP_HPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a */
