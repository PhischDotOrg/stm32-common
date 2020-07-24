/*-
 * $Copyright$
-*/

#ifndef _TIMER_CHANNEL_STM32F4_HPP_e6ed655f_fdd5_49a0_8fe0_c53a4107684f
#define _TIMER_CHANNEL_STM32F4_HPP_e6ed655f_fdd5_49a0_8fe0_c53a4107684f

#include <timer/TimerViaSTM32F4.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if 0  /* TODO Should this be !defined(HOSTBUILD)? */
#include <FreeRTOS.h>
#include <semphr.h>
#endif

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>
#include <stddef.h>

namespace timer {

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT = TimerViaSTM32F4>
class TimerChannelViaSTM32F4BaseT {
public:
    typedef enum TimerT::TimerChannel_e TimerChannel_t;
    typedef enum TimerT::TimerOutputCompareMode_e TimerOutputCompareMode_t;

protected:
    TimerChannelViaSTM32F4BaseT(TimerT &p_timer, const TimerChannel_t p_channel)
      : m_timer(p_timer), m_channel(p_channel) {
    }
    ~TimerChannelViaSTM32F4BaseT() {
    }
    
    TimerT &                m_timer;
    const TimerChannel_t    m_channel;
};

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT = TimerViaSTM32F4>
class TimerOutputChannelViaSTM32F4BaseT : public TimerChannelViaSTM32F4BaseT<TimerT> {
public:
    typedef typename TimerChannelViaSTM32F4BaseT<TimerT>::TimerChannel_t TimerChannel_t;
    typedef typename TimerChannelViaSTM32F4BaseT<TimerT>::TimerOutputCompareMode_t TimerOutputCompareMode_t;
    typedef TimerT Timer_t;
    
    void setup(const uint16_t p_pulse, const TimerOutputCompareMode_t p_mode) const;
    void enable(const bool p_activeLow = false) const;
    void disable(void) const;

protected:
    TimerOutputChannelViaSTM32F4BaseT(TimerT &p_timer, const TimerChannel_t p_channel);
    ~TimerOutputChannelViaSTM32F4BaseT();
};

// typedef TimerOutputChannelViaSTM32F4BaseT<> TimerOutputChannelViaSTM32F4;

/*******************************************************************************
 * 
 ******************************************************************************/
template<timer::TimerViaSTM32F4::TimerChannel_t ChannelT, typename TimerT = TimerViaSTM32F4>
class TimerOutputChannelViaSTM32F4TimerT : public TimerOutputChannelViaSTM32F4BaseT<TimerT> {
protected:
    TimerOutputChannelViaSTM32F4TimerT(TimerT &p_timer) : TimerOutputChannelViaSTM32F4BaseT<TimerT>(p_timer, ChannelT) {
    };

    ~TimerOutputChannelViaSTM32F4TimerT() {
    };
};

/*******************************************************************************
 * 
 ******************************************************************************/
template<intptr_t TimerT, timer::TimerViaSTM32F4::TimerChannel_t ChannelT>
class TimerOutputChannelViaSTM32F4T : public TimerOutputChannelViaSTM32F4TimerT<ChannelT, TimerViaSTM32F4T<TimerT> > {
public:
    TimerOutputChannelViaSTM32F4T(TimerViaSTM32F4T<TimerT> &p_timer)
      : TimerOutputChannelViaSTM32F4TimerT< ChannelT, TimerViaSTM32F4T<TimerT> >(p_timer) {
    };

    ~TimerOutputChannelViaSTM32F4T() {
    };
};

#if 0
typedef TimerOutputChannelViaSTM32F4TimerT<timer::TimerViaSTM32F4::e_TimerViaSTM32F4_Channel1> TimerOutputChannelViaSTM32F4_Channel1;
typedef TimerOutputChannelViaSTM32F4TimerT<timer::TimerViaSTM32F4::e_TimerViaSTM32F4_Channel2> TimerOutputChannelViaSTM32F4_Channel2;
typedef TimerOutputChannelViaSTM32F4TimerT<timer::TimerViaSTM32F4::e_TimerViaSTM32F4_Channel3> TimerOutputChannelViaSTM32F4_Channel3;
typedef TimerOutputChannelViaSTM32F4TimerT<timer::TimerViaSTM32F4::e_TimerViaSTM32F4_Channel4> TimerOutputChannelViaSTM32F4_Channel4;
#endif

};

#include "TimerChannelViaSTM32F4.cpp"

#endif /* _TIMER_CHANNEL_STM32F4_HPP_e6ed655f_fdd5_49a0_8fe0_c53a4107684f */
