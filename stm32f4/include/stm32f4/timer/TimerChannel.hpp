/*-
 * $Copyright$
-*/

#ifndef _TIMER_CHANNEL_HPP_649d0aa0_3e2e_494d_8453_7767cf811b5a
#define _TIMER_CHANNEL_HPP_649d0aa0_3e2e_494d_8453_7767cf811b5a

#if defined(DEBUG_BUILD) && defined(GMOCK_FOUND)
#include <gmock/gmock.h>
#endif /* defined(DEBUG_BUILD) && defined(GMOCK_FOUND) */

#include <timer/Timer.hpp>
#include <timer/TimerChannelViaSTM32F4.hpp>

/*******************************************************************************
 * 
 ******************************************************************************/
namespace timer {

template<typename TimerT> struct TimerChannelChoice;

template<> struct TimerChannelChoice<TimerViaSTM32F4> {
    typedef TimerChannelViaSTM32F4BaseT<TimerViaSTM32F4>        m_type;
    typedef TimerOutputChannelViaSTM32F4BaseT<TimerViaSTM32F4>  m_outputType;
};

template<> struct TimerChannelChoice<void> {
    typedef void m_type;
    typedef void m_outputType;
};

struct TimerChannelChoiceT {
    typedef TimerChannelChoice< timer::Timer >::m_type m_type;
    typedef TimerChannelChoice< timer::Timer >::m_outputType m_outputType;
};

typedef TimerChannelChoiceT::m_type TimerChannel;
typedef TimerChannelChoiceT::m_outputType TimerOutputChannel;

} /* namespace timer */

#endif /* _TIMER_CHANNEL_HPP_649d0aa0_3e2e_494d_8453_7767cf811b5a */
