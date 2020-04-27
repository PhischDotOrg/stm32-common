/*-
 * $Copyright$
-*/

#ifndef _GPIOSAMPLER_HPP_5A4683B7_7AA9_49E6_9D61_A3FEFF046672
#define _GPIOSAMPLER_HPP_5A4683B7_7AA9_49E6_9D61_A3FEFF046672

#include <tasks/Task.hpp>
#include <stdint.h>

namespace tasks {

template<typename UartT, typename PinT>
class GpioSamplerT : public Task {
private:
    UartT &         m_uart;
    PinT &          m_pin;
    const unsigned  m_periodMs;
    uint8_t         m_register;
    bool            m_pinState;
    
    virtual void run(void);

public:
    GpioSamplerT(const char * const p_name, UartT &p_uart, PinT &p_pin, const unsigned p_priority, const unsigned p_periodMs = 1000);
    virtual ~GpioSamplerT();
};

}; /* namespace tasks */

#include <tasks/GpioSampler.cpp>

#endif /* _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2 */
