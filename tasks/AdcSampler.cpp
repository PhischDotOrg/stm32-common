/*-
 * $Copyright$
-*/

#ifndef _ADC_SAMPLER_CPP_78536c9b_c083_4cea_88f8_adcf27e771dd
#define _ADC_SAMPLER_CPP_78536c9b_c083_4cea_88f8_adcf27e771dd

#include "tasks/AdcSampler.hpp"

#include "stm32f4xx.h"

#include <assert.h>
#include <string.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/semphr.h"
#include <printf.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {

template<typename UartT, typename PinT, typename AdcT, typename TimerChannelT>
const char * const
AdcSamplerT<UartT, PinT, AdcT, TimerChannelT>::m_name = "adcsampler";

#define USE_TIMER

/*******************************************************************************
 *
 ******************************************************************************/
template<typename TimerChannelT, typename UartT, typename PinT, typename AdcT>
AdcSamplerT<TimerChannelT, UartT, PinT, AdcT>::AdcSamplerT(UartT &p_uart, PinT &p_adcPin, AdcT &p_adc, TimerChannelT &p_timerChannel)
  : Task(m_name, 2, 1024), m_uart(p_uart), m_adcPin(p_adcPin), m_adc(p_adc), m_timerChannel(p_timerChannel) {
    m_adcPin.enable(gpio::GpioAccessViaSTM32F4::e_Analog, gpio::GpioAccessViaSTM32F4::e_None, gpio::GpioAccessViaSTM32F4::e_Gpio);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename TimerChannelT, typename UartT, typename PinT, typename AdcT>
AdcSamplerT<TimerChannelT, UartT, PinT, AdcT>::~AdcSamplerT() {
    m_adcPin.disable();
}

/*******************************************************************************
 *
 ******************************************************************************/
static uint32_t buffer[8];

template<typename TimerChannelT, typename UartT, typename PinT, typename AdcT>
void
AdcSamplerT<TimerChannelT, UartT, PinT, AdcT>::run(void) {
    m_uart.printf("Task '%s' starting...\r\n", this->m_name);


#if defined(USE_TIMER)
    /*
     * This sets up the timer channel to toggle whenever the counter value reaches
     * zero. In effect, this means the timer channel output toggles with double
     * the period of the timer overflow.
     */
    this->m_timerChannel.setup(0, TimerChannelT::Timer_t::e_TimerViaSTM32F4_OCM_Toggle);
#endif
    
    this->m_adc.setupResolution(AdcT::e_AdcResolution_12Bit);
    this->m_adc.setupChannel(AdcT::e_AdcChannel_9, AdcT::e_AdcRank_1, AdcT::e_ADC_SampleTime_15Cycles);

    while (1) {
        m_uart.printf("%s: ", this->m_name);
        for (unsigned i = 0; i < sizeof(buffer) / sizeof(buffer[0]); i++)
        {
            m_uart.printf("[0x%x] ", buffer[i]);
        }
        m_uart.printf("\r\n");

        m_uart.printf("Task '%s' sleep...\r\n", this->m_name);
        vTaskDelay(1000 / portTICK_RATE_MS);

        m_uart.printf("Task '%s' sampling...\r\n", this->m_name);

#if defined(USE_TIMER)
        this->m_adc.setupExternalTrigger(this->m_timerChannel, AdcT::e_AdcTriggerEnable_BothEdges);
        this->m_timerChannel.enable();
        unsigned rc = m_adc.sample(buffer, sizeof(buffer[0]), sizeof(buffer) / sizeof(buffer[0]));
        this->m_timerChannel.disable();
#else
        unsigned rc = m_adc.sample(buffer);
#endif
        if (rc) {
            m_uart.printf("Task '%s': Sampling failed with rc=0x%x\r\n", this->m_name, rc);
        }
    }
}

} /* namespace tasks */

#endif /* _ADC_SAMPLER_CPP_78536c9b_c083_4cea_88f8_adcf27e771dd */
