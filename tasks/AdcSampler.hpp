/*-
 * $Copyright$
-*/

#ifndef _ADC_SAMPLER_HPP_9229553b_5236_4c63_acc6_56525d6981b7
#define _ADC_SAMPLER_HPP_9229553b_5236_4c63_acc6_56525d6981b7

#include "tasks/Task.hpp"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/semphr.h"

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {

template<typename TimerChannelT, typename UartT = uart::UartDevice, typename PinT = gpio::GpioPin,
  typename AdcT = devices::AdcViaSTM32F4>
class AdcSamplerT : public Task {
private:
    static const char * const   m_name;
    UartT &                     m_uart;
    PinT &                      m_adcPin;
    AdcT &                      m_adc;
    TimerChannelT &             m_timerChannel;

    virtual void run(void);

public:
    AdcSamplerT(UartT &p_uart, PinT &p_adcPin, AdcT &p_adc, TimerChannelT &p_timer);
    virtual ~AdcSamplerT();
};

}; /* namespace tasks */

#include "AdcSampler.cpp"

#endif /* _ADC_SAMPLER_HPP_9229553b_5236_4c63_acc6_56525d6981b7 */
