/*-
 * $Copyright$
-*/

#ifndef _TIMER_STM32F4_HPP_d8a8abac_6955_43d9_98ef_c8730935c822
#define _TIMER_STM32F4_HPP_d8a8abac_6955_43d9_98ef_c8730935c822

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <FreeRTOS.h>
#include <semphr.h>
    
#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>
#include <stddef.h>

#include <stm32f4/RccViaSTM32F4.hpp>
#include <gpio/GpioAccessViaSTM32F4.hpp>
#include <gpio/GpioPin.hpp>

namespace timer {

/*******************************************************************************
 * 
 ******************************************************************************/
class TimerViaSTM32F4 {    
public:
    typedef enum TimerChannel_e {
        e_TimerViaSTM32F4_Channel1 = 0,
        e_TimerViaSTM32F4_Channel2 = 1,
        e_TimerViaSTM32F4_Channel3 = 2,
        e_TimerViaSTM32F4_Channel4 = 3,
    } TimerChannel_t;

    typedef enum TimerOutputCompareMode_e {
        e_TimerViaSTM32F4_OCM_Frozen        = 0x0,
        e_TimerViaSTM32F4_OCM_Active        = 0x1,
        e_TimerViaSTM32F4_OCM_Inactive      = 0x2,
        e_TimerViaSTM32F4_OCM_Toggle        = 0x3,
        e_TimerViaSTM32F4_OCM_ForceInactive = 0x4,
        e_TimerViaSTM32F4_OCM_ForceActive   = 0x5,
        e_TimerViaSTM32F4_OCM_PwmMode1      = 0x6,
        e_TimerViaSTM32F4_OCM_PwmMode2      = 0x7,
    } TimerOutputCompareMode_t;
    
    typedef enum ClockDivider_e {
        e_TimerViaSTM32F4_ClkDiv0 = 0x0,
        e_TimerViaSTM32F4_ClkDiv2 = 0x1,
        e_TimerViaSTM32F4_ClkDiv4 = 0x2,
    } ClockDivider_t;

    void setup(const uint16_t p_prescaler, const uint16_t p_period);

    void enable(void) const;
    void disable(void) const;

    void setupOutputChannel(const TimerChannel_t p_channel, const uint16_t p_pulse, const TimerOutputCompareMode_t p_mode) const;

    void enableOutputChannel(const TimerChannel_t p_channel, const bool p_activeLow = false) const;
    void disableOutputChannel(const TimerChannel_t p_channel) const;

    void handleIrq(void) const;

    void enableUpdateIrq(void) const;
    void disableUpdateIrq(void) const;

protected:
    TimerViaSTM32F4(TIM_TypeDef &p_timer);
    ~TimerViaSTM32F4();

private:
    TIM_TypeDef &m_timer;

    void setClockDivider(const ClockDivider_t p_divider) const;
    void setPrescaler(const uint16_t p_prescaler) const;
    void setPeriod(const uint16_t p_period) const;
};

/*******************************************************************************
 * 
 ******************************************************************************/
template<intptr_t> struct TimerViaSTM32F4FunctionHelper;

#if defined(TIM1_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM1_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB2_t m_rcc = devices::RccViaSTM32F4::e_Tim1;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim1;
};
#endif

#if defined(TIM2_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM2_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim2;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim2;
};
#endif

#if defined(TIM3_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM3_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim3;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim3;
};
#endif

#if defined(TIM4_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM4_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim4;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim4;
};
#endif

#if defined(TIM5_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM5_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim5;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim5;
};
#endif

#if defined(TIM6_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM6_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim6;
};
#endif

#if defined(TIM7_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM7_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim7;
};
#endif

#if defined(TIM8_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM8_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB2_t m_rcc = devices::RccViaSTM32F4::e_Tim8;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim8;
};
#endif

#if defined(TIM9_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM9_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB2_t m_rcc = devices::RccViaSTM32F4::e_Tim9;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim9;
};
#endif

#if defined(TIM10_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM10_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB2_t m_rcc = devices::RccViaSTM32F4::e_Tim10;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim10;
};
#endif

#if defined(TIM11_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM11_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB2_t m_rcc = devices::RccViaSTM32F4::e_Tim11;
    static const gpio::GpioAccessViaSTM32F4::Function_t m_gpio = gpio::GpioAccessViaSTM32F4::e_Tim11;
};
#endif

#if defined(TIM12_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM12_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim12;
};
#endif

#if defined(TIM13_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM13_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim13;
};
#endif

#if defined(TIM14_BASE)
template<> struct TimerViaSTM32F4FunctionHelper<TIM14_BASE> {
    static const devices::RccViaSTM32F4::FunctionAPB1_t m_rcc = devices::RccViaSTM32F4::e_Tim14;
};
#endif

/*******************************************************************************
 * 
 ******************************************************************************/
template<intptr_t TimerT, typename RccT = devices::RccViaSTM32F4>
class TimerViaSTM32F4RccT : public TimerViaSTM32F4 {
public:
    TimerViaSTM32F4RccT(RccT &p_rcc) : TimerViaSTM32F4(* reinterpret_cast<TIM_TypeDef *>(TimerT)), m_rcc(p_rcc) {
        m_rcc.enable(TimerViaSTM32F4FunctionHelper<TimerT>::m_rcc);
    }
    
    ~TimerViaSTM32F4RccT(void) {
        m_rcc.disable(TimerViaSTM32F4FunctionHelper<TimerT>::m_rcc);
    }

    void setOutputPin(gpio::PinT<gpio::EngineT<gpio::GpioAccessViaSTM32F4> > &p_pin) const {
        p_pin.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, TimerViaSTM32F4FunctionHelper<TimerT>::m_gpio);
    }

private:
    RccT &m_rcc;
};

template <intptr_t TimerT>
class TimerViaSTM32F4T : public TimerViaSTM32F4RccT<TimerT> {
public:
    TimerViaSTM32F4T(devices::RccViaSTM32F4 &p_rcc)
      : TimerViaSTM32F4RccT<TimerT>(p_rcc) {
          
    };
    ~TimerViaSTM32F4T() {
        
    };
};

/*******************************************************************************
 * 
 ******************************************************************************/
#if defined(TIM1_BASE)
typedef TimerViaSTM32F4T<TIM1_BASE> TimerViaSTM32F4_Tim1;
#endif
#if defined(TIM2_BASE)
typedef TimerViaSTM32F4T<TIM2_BASE> TimerViaSTM32F4_Tim2;
#endif
#if defined(TIM3_BASE)
typedef TimerViaSTM32F4T<TIM3_BASE> TimerViaSTM32F4_Tim3;
#endif
#if defined(TIM4_BASE)
typedef TimerViaSTM32F4T<TIM4_BASE> TimerViaSTM32F4_Tim4;
#endif
#if defined(TIM5_BASE)
typedef TimerViaSTM32F4T<TIM5_BASE> TimerViaSTM32F4_Tim5;
#endif
#if defined(TIM6_BASE)
typedef TimerViaSTM32F4T<TIM6_BASE> TimerViaSTM32F4_Tim6;
#endif
#if defined(TIM7_BASE)
typedef TimerViaSTM32F4T<TIM7_BASE> TimerViaSTM32F4_Tim7;
#endif
#if defined(TIM8_BASE)
typedef TimerViaSTM32F4T<TIM8_BASE> TimerViaSTM32F4_Tim8;
#endif
#if defined(TIM9_BASE)
typedef TimerViaSTM32F4T<TIM9_BASE> TimerViaSTM32F4_Tim9;
#endif
#if defined(TIM10_BASE)
typedef TimerViaSTM32F4T<TIM10_BASE> TimerViaSTM32F4_Tim10;
#endif
#if defined(TIM11_BASE)
typedef TimerViaSTM32F4T<TIM11_BASE> TimerViaSTM32F4_Tim11;
#endif
#if defined(TIM12_BASE)
typedef TimerViaSTM32F4T<TIM12_BASE> TimerViaSTM32F4_Tim12;
#endif
#if defined(TIM13_BASE)
typedef TimerViaSTM32F4T<TIM13_BASE> TimerViaSTM32F4_Tim13;
#endif
#if defined(TIM14_BASE)
typedef TimerViaSTM32F4T<TIM14_BASE> TimerViaSTM32F4_Tim14;
#endif

} /* namespace timer */

#endif /* _TIMER_STM32F4_HPP_d8a8abac_6955_43d9_98ef_c8730935c822 */
