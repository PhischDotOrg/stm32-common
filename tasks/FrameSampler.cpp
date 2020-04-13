/*-
 * $Copyright$
-*/

#include "tasks/FrameSampler.hpp"

#ifndef _FRAMESAMPLER_CPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a
#define _FRAMESAMPLER_CPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a

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
template<typename BufferT, size_t N, class TimerChannelT, class AdcT, typename UartT>
FrameSamplerT<BufferT, N, TimerChannelT, AdcT, UartT>::FrameSamplerT(const char * const p_name, const unsigned p_priority,
 UartT &p_uart, BufferT (&p_frames)[N], TimerChannelT &p_timerChannel, AdcT &p_adc, const unsigned p_periodMs)
  : Task(p_name, p_priority), m_uart(p_uart), m_frames(p_frames), m_timerChannel(p_timerChannel), m_adc(p_adc),
    m_current(0), m_periodMs(p_periodMs), m_txQueue(NULL) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename BufferT, size_t N, class TimerChannelT, class AdcT, typename UartT>
FrameSamplerT<BufferT, N, TimerChannelT, AdcT, UartT>::~FrameSamplerT() {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename BufferT, size_t N, class TimerChannelT, class AdcT, typename UartT>
void
FrameSamplerT<BufferT, N, TimerChannelT, AdcT, UartT>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t period = (this->m_periodMs * 1000) / portTICK_RATE_MICROSECONDS;

    /* Can't do this in the constructor; FreeRTOS must be running */
    for (unsigned i = 0; i < N; i++) {
        this->m_frames[i].unlock();
    }
    
     /*
     * This sets up the timer channel to toggle whenever the counter value reaches
     * zero. In effect, this means the timer channel output toggles with double
     * the period of the timer overflow.
     */
    this->m_timerChannel.setup(0, TimerChannelT::Timer_t::e_TimerViaSTM32F4_OCM_Toggle);

    this->m_adc.setupResolution(AdcT::e_AdcResolution_8Bit);
    this->m_adc.setupChannel(AdcT::e_AdcChannel_9, AdcT::e_AdcRank_1, AdcT::e_ADC_SampleTime_3Cycles);
    this->m_adc.setupExternalTrigger(AdcT::e_AdcTrigger_Timer4_CC4, AdcT::e_AdcTriggerEnable_BothEdges);

    this->m_timerChannel.enable();

    for (BufferT *buffer = &this->m_frames[0]; ; buffer = &this->m_frames[++this->m_current % N]) {
        /*
         * This ensures a constant wait time between loop executions. See the
         * documentation of vTaskDelayUntil() for details.
         */
        if (this->m_periodMs)
            vTaskDelayUntil(&lastWakeTime, period);

        if (buffer->lock() != 0) {
            this->m_uart.printf("%s(): Failed to lock buffer %u. Aborting!\r\n",
              this->m_name, this->m_current % N);
            break;
        };

        int rc = this->m_adc.sample(buffer->data(), sizeof(uint16_t), buffer->size(), this->m_periodMs);
    
        if (buffer->unlock() != 0) {
            this->m_uart.printf("%s(): Failed to unlock buffer %u. Aborting!\r\n",
              this->m_name, this->m_current % N);
            break;
        }

        if (rc == 0) {
            /*
             * This copies n bytes from &buffer into the queue. The value n is
             * configured as sizeof(BufferT*) when the queue was created). So
             * basically, this copies the contents of the buffer pointer to the
             * queue which, in effect, writes a BufferT * to the queue.
             */
            if (xQueueSend(*this->m_txQueue, &buffer, 0) != pdTRUE) {
                this->m_uart.printf("%s(): Failed to post buffer %d to queue. Aborting!\r\n",
                  this->m_name, this->m_current % N);
                break;
            }
        } else {
            this->m_uart.printf("%s(): Failed to fill frame buffer with samples!\r\n", this->m_name);
        }
    };

    this->m_timerChannel.disable();
    
    this->m_uart.printf("%s() ended!\r\n", this->m_name);
    halt(__FILE__, __LINE__);
}

} /* namespace tasks */

#endif /* _FRAMESAMPLER_CPP_aa6ac2c3_3489_4a38_8642_30cdd3943a2a */
