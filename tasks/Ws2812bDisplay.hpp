/*-
 * $Copyright$
-*/

#ifndef _WS2812B_DISPLAY_HPP_839ff64f_0d20_4320_9dc5_ccce2f9ca4be
#define _WS2812B_DISPLAY_HPP_839ff64f_0d20_4320_9dc5_ccce2f9ca4be

#include "tasks/Task.hpp"

#include <gpio/GpioPin.hpp>
#include <uart/UartDevice.hpp>

namespace tasks {

template<typename InputBufferT, typename Ws2812bT, typename UartT = uart::UartDevice>
class Ws2812bDisplayT : public Task {
public:
    Ws2812bDisplayT(const char * const p_name, UartT &p_uart, Ws2812bT &p_ws2812b, const unsigned p_priority);
    virtual ~Ws2812bDisplayT();

    void setRxQueue(QueueHandle_t* p_rxQueue) {
        m_rxQueue = p_rxQueue;
    }

    QueueHandle_t* getRxQueue() const {
        return m_rxQueue;
    }

    static const unsigned   WIDTH   = 20;
    static const unsigned   HEIGHT  = 15;

private:
    Ws2812bT        m_ws2812b;
    UartT &         m_uart;
    QueueHandle_t * m_rxQueue;

    const typename Ws2812bT::Rgb_t m_off;
    const typename Ws2812bT::Rgb_t m_on;
          typename Ws2812bT::Rgb_t m_amber;
    const typename Ws2812bT::Rgb_t m_red;

    const unsigned  m_amberThreshold;
    const unsigned  m_redThreshold;

    static const float m_min[WIDTH];
    static const float m_max[WIDTH];
    
    float   m_factors[WIDTH][HEIGHT];
    
    constexpr static unsigned   m_lowPassFilterDepth = 5;
    unsigned                    m_lowPassFilterIndex;
    float m_lowPassFilter[WIDTH][m_lowPassFilterDepth];
    
    virtual void run(void);
    
    void setupStraightBar(const unsigned p_bar, const unsigned p_value);
    void setupReversedBar(const unsigned p_bar, const unsigned p_value);
    void setupThresholds(void);
};

}; /* namespace tasks */

#include "Ws2812bDisplay.cpp"

#endif /* _WS2812B_DISPLAY_HPP_839ff64f_0d20_4320_9dc5_ccce2f9ca4be */
