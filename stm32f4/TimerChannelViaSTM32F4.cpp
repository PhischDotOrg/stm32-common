/*-
 * $Copyright$
-*/

#ifndef _TIMER_CHANNEL_STM32F4_HPP_a1c883c0_135c_487e_ade6_41a1e6ecdb24
#define _TIMER_CHANNEL_STM32F4_HPP_a1c883c0_135c_487e_ade6_41a1e6ecdb24

#include <timer/TimerChannelViaSTM32F4.hpp>

namespace timer {
    
/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT>
TimerOutputChannelViaSTM32F4BaseT<TimerT>::TimerOutputChannelViaSTM32F4BaseT(TimerT &p_timer, const TimerChannel_t p_channel)
  : TimerChannelViaSTM32F4BaseT<TimerT>(p_timer, p_channel) {

}

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT>
TimerOutputChannelViaSTM32F4BaseT<TimerT>::~TimerOutputChannelViaSTM32F4BaseT() {
    
}

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT>
void
TimerOutputChannelViaSTM32F4BaseT<TimerT>::setup(const uint16_t p_pulse, const TimerOutputCompareMode_t p_mode) const {
    this->m_timer.setupOutputChannel(this->m_channel, p_pulse, p_mode);    
}

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT>
void
TimerOutputChannelViaSTM32F4BaseT<TimerT>::enable(const bool p_activeLow /* = false */) const {
    this->m_timer.enableOutputChannel(this->m_channel, p_activeLow);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template <typename TimerT>
void
TimerOutputChannelViaSTM32F4BaseT<TimerT>::disable(void) const {
    this->m_timer.disableOutputChannel(this->m_channel);
}

} /* namespace timer */

#endif /* _TIMER_CHANNEL_STM32F4_HPP_a1c883c0_135c_487e_ade6_41a1e6ecdb24 */