/*-
 * $Copyright$
-*/

#ifndef _WS2812B_DISPLAY_CPP_c9be6f0e_bbb2_4063_a3a5_b116ec616f16
#define _WS2812B_DISPLAY_CPP_c9be6f0e_bbb2_4063_a3a5_b116ec616f16

#include "Ws2812bDisplay.hpp"

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

#include "uart/Uart.hpp"

#include <algorithm>
#include <cmath>

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
const float
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::m_min[WIDTH] = {
    0,      0,      0,      0,      0,
    0,      0,      0,      0,      0,
    0,      0,      0,      0,      0,
    0,      0,      0,      0,      0,
};

#if 0
template<typename InputBufferT, typename Ws2812bT, typename UartT>
const float
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::m_max[WIDTH] = {
    512,    512,    512,    512,    511,
    510,    510,    508,    505,    505,
    505,    500,    495,    490,    485,
    475,    470,    460,    440,    400,
};
#endif

template<typename InputBufferT, typename Ws2812bT, typename UartT>
const float
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::m_max[WIDTH] = {
    132,    127,    122,    117,    112,
    112,    111,    111,    111,    111,
    110,    109,    108,    107,    106,
    105,    90,    80,     70,     60,
};

#if 0
template<typename InputBufferT, typename Ws2812bT, typename UartT>
const float
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::m_max[WIDTH] = {
     33,     32,     31,     30,     28,
     28,     28,     28,     28,     28,
     28,     27,     27,     27,     27,
     27,     23,     20,     17,     15,
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::Ws2812bDisplayT(const char * const p_name, UartT &p_uart,
 Ws2812bT &p_ws2812b, const unsigned p_priority)
  : Task(p_name, p_priority, configMINIMAL_STACK_SIZE * 4), m_ws2812b(p_ws2812b), m_uart(p_uart), m_rxQueue(NULL),
    m_off(0, 0, 0), m_on(0, 32, 0), m_amber(0xff, 0xbf, 0x00), m_red(32, 0, 0),
    m_amberThreshold(10), m_redThreshold(13)
{
    this->setupThresholds();
    
    m_amber %= 50;
    
    m_lowPassFilterIndex = 0;
    for (unsigned i = 0; i < m_lowPassFilterDepth; i++) {
        for (unsigned band = 0; band < WIDTH; band++) {
            m_lowPassFilter[band][i] = 0;
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::~Ws2812bDisplayT() {
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
void
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::setupStraightBar(const unsigned p_bar, const unsigned p_value) {
    unsigned end   = (this->m_ws2812b.size() - 1) - (p_bar * HEIGHT) + 1;
    unsigned start = end - HEIGHT;

    for (unsigned pixel = start + 0; pixel < (start + p_value); pixel++) {
        if ((pixel - start) >= this->m_redThreshold) {
            this->m_ws2812b.setPixel(pixel, this->m_red);
        } else if ((pixel - start) >= this->m_amberThreshold) {
            this->m_ws2812b.setPixel(pixel, this->m_amber);
        } else {
            this->m_ws2812b.setPixel(pixel, this->m_on);
        }
    }
    for (unsigned pixel = start + p_value; pixel < end; pixel++) {
        this->m_ws2812b.setPixel(pixel, this->m_off);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
void
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::setupReversedBar(const unsigned p_bar, const unsigned p_value) {
    unsigned start = (this->m_ws2812b.size() - 1) - (p_bar * HEIGHT);
    unsigned end    = start - HEIGHT;

    for (unsigned pixel = start - 0; pixel > (start - p_value); pixel--) {
        if ((start - pixel) >= this->m_redThreshold) {
            this->m_ws2812b.setPixel(pixel, this->m_red);
        } else if ((start - pixel) >= this->m_amberThreshold) {
            this->m_ws2812b.setPixel(pixel, this->m_amber);
        } else {
            this->m_ws2812b.setPixel(pixel, this->m_on);
        }
    }
    for (unsigned pixel = start - p_value; pixel > end; pixel--) {
        this->m_ws2812b.setPixel(pixel, this->m_off);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
void
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::run(void) {
    this->m_uart.printf("Task '%s' starting...\r\n", this->m_name);

    static_assert(InputBufferT::SIZE <= WIDTH, "Display not wide enough for input buffer");
    static_assert(Ws2812bT::SIZE >= (WIDTH * HEIGHT), "LED Strip not long enough for size of display");

    InputBufferT *buffer;

    while (1) {
        if (xQueueReceive(*this->m_rxQueue, &buffer, portMAX_DELAY) != pdTRUE) {
            this->m_uart.printf("%s(): Failed to receive buffer from queue. Aborting!\r\n", this->m_name);
            break;
        }

        if (buffer->lock() != 0) {
            this->m_uart.printf("%s(): Failed to lock buffer. Aborting!\r\n", this->m_name);
            break;
        };
        
        m_lowPassFilterIndex = (m_lowPassFilterIndex + 1) % m_lowPassFilterDepth;
        unsigned bar = 0;
        for (typename InputBufferT::const_iterator it = buffer->begin(); it != buffer->end(); it++) {
            m_lowPassFilter[bar][m_lowPassFilterIndex] = *it;
            
            float value = 0, mul = 1.0f / (m_lowPassFilterDepth + 1);
            for (unsigned count = 0; count < m_lowPassFilterDepth; count++) {
                value += mul * m_lowPassFilter[bar][count];
            }
            value += 1 * mul * (*it);

            value = std::max(value, *it);

            unsigned idx = 0;
            while ((idx < (sizeof(this->m_factors[bar]) / sizeof(this->m_factors[bar][0])))
              && (value > this->m_factors[bar][idx]))
                idx++;

#if defined(DEBUG_WS2812B_DISPLAY)
            m_uart.printf("%s(): %d: %d\r\n", this->m_name, bar, idx);
#endif /* defined(DEBUG_WS2812B_DISPLAY) */
            
            if ((bar % 2) == 0) { /* Upside Down Bars */
                this->setupReversedBar(bar, idx);
            } else { /* Straight Up Bar */
                this->setupStraightBar(bar, idx);
            }

            bar = (bar + 1) % WIDTH;
        }

        if (buffer->unlock() != 0) {
            this->m_uart.printf("%s(): Failed to unlock buffer %i. Aborting!\r\n", this->m_name);
            break;
        }
        
        this->m_ws2812b.show();
    }

    this->m_uart.printf("%s() ended!\r\n", this->m_name);
    halt(__FILE__, __LINE__);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename InputBufferT, typename Ws2812bT, typename UartT>
void
Ws2812bDisplayT<InputBufferT, Ws2812bT, UartT>::setupThresholds(void) {
    const unsigned nFactors = sizeof(this->m_factors[0]) / sizeof(this->m_factors[0][HEIGHT]);
    assert(nFactors == HEIGHT);

#define WS2812B_LOG_SCALE
#if defined(WS2812B_LOG_SCALE)
    for (unsigned w = 0; w < WIDTH; w++) {
        const float offs = this->m_min[w];
        const float logFactor = std::log(std::pow(this->m_max[w] - offs, 1.0f / static_cast<float>(m_redThreshold-1)));
        for (unsigned i = 0; i < nFactors; i++) {
            this->m_factors[w][i] = offs + std::exp((i + 1) * logFactor);
        }
    }
#else /* !defined(WS2812B_LOG_SCALE) */
    for (unsigned w = 0; w < WIDTH; w++) {
        const float offs = this->m_min[w];
        const float linFactor = ((this->m_max[w] / 2) - offs) / static_cast<float>(m_redThreshold-1);
        for (unsigned i = 0; i < nFactors; i++) {
            this->m_factors[w][i] = offs + ((i + 1) * linFactor);
        }
    }    
#endif  /* defined(WS2812B_LOG_SCALE) */
}

} /* namespace tasks */

#endif /* _WS2812B_DISPLAY_CPP_c9be6f0e_bbb2_4063_a3a5_b116ec616f16 */