/*-
 * $Copyright$
-*/

#ifndef _FRAME_DISPLAYER_CPP_50f9b608_9fd9_45ed_8ae1_4460fbe33a20
#define _FRAME_DISPLAYER_CPP_50f9b608_9fd9_45ed_8ae1_4460fbe33a20

#include <tasks/FrameDisplayer.hpp>

#include "FrameDisplayer.hpp"

#include <type_traits>

namespace tasks {
    
/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename DisplayT, typename UartT>
FrameDisplayerT<InputBufferT, DisplayT, UartT>::FrameDisplayerT(const char* const p_name,
  const unsigned p_priority, UartT& p_uart, DisplayT &p_display)
    : Task(p_name, p_priority), m_uart(p_uart), m_display(p_display), m_rxQueue(NULL) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename DisplayT, typename UartT>
FrameDisplayerT<InputBufferT, DisplayT, UartT>::~FrameDisplayerT() {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
static unsigned factors[8] = { 16, 64, 256, 1024, 2048, 4096, 8192, 12288 };
static uint8_t values[8] = { 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF };

template<typename InputBufferT, typename DisplayT, typename UartT>
void
FrameDisplayerT<InputBufferT, DisplayT, UartT>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);
    InputBufferT *buffer;

    static_assert((InputBufferT::SIZE - 2) <= DisplayT::m_width, "Display not wide enough for number of bars");
            
    while (1) {
        if (xQueueReceive(*this->m_rxQueue, &buffer, portMAX_DELAY) != pdTRUE) {
            this->m_uart.printf("%s(): Failed to receive buffer from queue. Aborting!\r\n", this->m_name);
            break;
        }

        if (buffer->lock() != 0) {
            this->m_uart.printf("%s(): Failed to lock buffer. Aborting!\r\n", this->m_name);
            break;
        };

        typename InputBufferT::const_iterator it = buffer->begin() + 1;
        for (unsigned column = 0; it != (buffer->end() - 1); it++, column++) {
            typename DisplayT::ColVector_t vector = -1;

            vector = 0;
            for (unsigned i = 0; i < sizeof(typename DisplayT::ColVector_t) * 8; i++) {
                if (*it > factors[i]) {
                    vector = values[i];
                }
            }

            this->m_display.setColumn(column, DisplayT::e_RgbChannel_Red, vector);
            this->m_display.setColumn(column, DisplayT::e_RgbChannel_Green, vector);
            this->m_display.setColumn(column, DisplayT::e_RgbChannel_Blue, vector);
        }
        
        if (buffer->unlock() != 0) {
            this->m_uart.printf("%s(): Failed to unlock buffer %i. Aborting!\r\n", this->m_name);
            break;
        }
    }

    this->m_uart.printf("%s() ended!\r\n", this->m_name);
    halt(__FILE__, __LINE__);
}

} /* namespace tasks */

#endif /* _FRAME_DISPLAYER_CPP_50f9b608_9fd9_45ed_8ae1_4460fbe33a20 */
