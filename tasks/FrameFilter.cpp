/*-
 * $Copyright$
-*/
#include "tasks/FrameSampler.hpp"

#ifndef _FRAME_FILTER_CPP_4085c3de_7e34_4002_a351_d24c2cd81aff
#define _FRAME_FILTER_CPP_4085c3de_7e34_4002_a351_d24c2cd81aff

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
template<typename InputBufferT, typename OutputBufferT, size_t N, typename FilterT, typename PinT, typename UartT>
FrameFilterT<InputBufferT, OutputBufferT, N, FilterT, PinT, UartT>::FrameFilterT(const char * const p_name,
  const unsigned p_priority, UartT &p_uart, OutputBufferT (&p_frames)[N], FilterT &p_filter)
    : Task(p_name, p_priority, configMINIMAL_STACK_SIZE * 2), m_uart(p_uart), m_frames(p_frames), m_filter(p_filter), m_activeIndicationPin(NULL),
        m_current(0), m_rxQueue(NULL), m_txQueue(NULL) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename OutputBufferT, size_t N, typename FilterChainT, typename PinT, typename UartT>
FrameFilterT<InputBufferT, OutputBufferT, N, FilterChainT, PinT, UartT>::~FrameFilterT() {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename OutputBufferT, size_t N, typename FilterChainT, typename PinT, typename UartT>
void
FrameFilterT<InputBufferT, OutputBufferT, N, FilterChainT, PinT, UartT>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);
    InputBufferT *rxBuffer;
    int rc = 0;

    /* Can't do this in the constructor; FreeRTOS must be running */
    for (unsigned i = 0; i < N; i++) {
        this->m_frames[i].unlock();
    }
    
    for (OutputBufferT *txBuffer = &this->m_frames[0]; ; txBuffer = &this->m_frames[++this->m_current % N]) {
        if (xQueueReceive(*this->m_rxQueue, &rxBuffer, portMAX_DELAY) != pdTRUE) {
            this->m_uart.printf("%s(): Failed to receive buffer from queue. Aborting!\r\n", this->m_name);
            break;
        }

#if defined(WITH_PROFILING)
        if (this->m_activeIndicationPin != NULL)
            this->m_activeIndicationPin->set(gpio::Pin::On);
#endif /* defined(WITH_PROFILING) */

        if (rxBuffer->lock() != 0) {
            this->m_uart.printf("%s(): Failed to lock Rx Buffer. Aborting!\r\n", this->m_name);
            break;
        };
        if (txBuffer->lock() != 0) {
            this->m_uart.printf("%s(): Failed to lock Buffer %u. Aborting!\r\n", this->m_name, this->m_current);
            break;
        };

        this->m_filter.filter(*rxBuffer, *txBuffer);

#if defined(DEBUG_FRAME_FILTER)
        unsigned idx = 0;
        for (typename OutputBufferT::const_iterator iter = txBuffer->begin(); iter != txBuffer->end(); iter++, idx++) {
            volatile unsigned value = ((*iter) * 1000);
            m_uart.printf("%s(): %d: %d\r\n", this->m_name, idx, (value + 500) / 1000);
        }
#endif /* defined(DEBUG_FRAME_FILTER) */
        
        if (txBuffer->unlock() != 0) {
            this->m_uart.printf("%s(): Failed to unlock Buffer %u. Aborting!\r\n", this->m_name, this->m_current);
            break;
        };
        if (rxBuffer->unlock() != 0) {
            this->m_uart.printf("%s(): Failed to unlock Rx Buffer. Aborting!\r\n", this->m_name);
            break;
        }

        if (rc == 0) {
            /*
             * This copies n bytes from &buffer into the queue. The value n is
             * configured as sizeof(OutputBufferT*) when the queue was created). So
             * basically, this copies the contents of the buffer pointer to the
             * queue which, in effect, writes a OutputBufferT * to the queue.
             */
            if (xQueueSend(*this->m_txQueue, &txBuffer, 0) != pdTRUE) {
                this->m_uart.printf("%s(): Failed to post buffer %d to queue. Aborting!\r\n",
                  this->m_name, this->m_current % N);
                break;
            }
        } else {
            this->m_uart.printf("%s(): Failed to fill frame buffer with samples!\r\n", this->m_name);
        }
#if defined(WITH_PROFILING)
        if (this->m_activeIndicationPin != NULL)
            this->m_activeIndicationPin->set(gpio::Pin::Off);
#endif /* defined(WITH_PROFILING) */
    }
    
    this->m_uart.printf("%s() ended!\r\n", this->m_name);
    halt(__FILE__, __LINE__);
}

} /* namespace tasks */

#endif /* _FRAME_FILTER_CPP_4085c3de_7e34_4002_a351_d24c2cd81aff */
