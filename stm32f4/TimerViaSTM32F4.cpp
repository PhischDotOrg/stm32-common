/*-
 * $Copyright$
-*/

#ifndef _TIMER_STM32F4_HPP_227fb640_e182_4eca_9f47_b665bea6ea0b
#define _TIMER_STM32F4_HPP_227fb640_e182_4eca_9f47_b665bea6ea0b

#include <timer/TimerViaSTM32F4.hpp>

namespace timer {
    
/*******************************************************************************
 * 
 ******************************************************************************/
TimerViaSTM32F4::TimerViaSTM32F4(TIM_TypeDef &p_timer) : m_timer(p_timer) {
}

/*******************************************************************************
 * 
 ******************************************************************************/
TimerViaSTM32F4::~TimerViaSTM32F4() {
}

/*******************************************************************************
 *
 ******************************************************************************/
void
TimerViaSTM32F4::setClockDivider(const ClockDivider_t p_divider) const {
    this->m_timer.CR1 &= ~TIM_CR1_CKD;
    this->m_timer.CR1 |= p_divider << 8;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
TimerViaSTM32F4::setPrescaler(const uint16_t p_prescaler) const {
    this->m_timer.PSC = p_prescaler;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
TimerViaSTM32F4::setPeriod(const uint16_t p_period) const {
    this->m_timer.ARR = p_period;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
TimerViaSTM32F4::setup(const uint16_t p_prescaler, const uint16_t p_period) {
    assert(p_prescaler > 0);

    this->setPrescaler(p_prescaler - 1);
    this->setPeriod(p_period);
    
    /* Generate Update Event to re-load all values internally to the hardware */
    this->m_timer.EGR |= TIM_EGR_UG;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
TimerViaSTM32F4::enable(void) const {
    this->m_timer.CR1 |= TIM_CR1_CEN;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
TimerViaSTM32F4::disable(void) const {
    this->m_timer.CR1 &= ~TIM_CR1_CEN;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
TimerViaSTM32F4::setupOutputChannel(const TimerChannel_t p_channel, const uint16_t p_pulse, const TimerOutputCompareMode_t p_mode) const {
    volatile uint16_t * const CCMR = (p_channel < 2) ? reinterpret_cast<volatile uint16_t *>(&(this->m_timer.CCMR1)) : reinterpret_cast<volatile uint16_t *>(&(this->m_timer.CCMR2));
    const unsigned shift = (p_channel % 2) ? 8 : 0;

    *CCMR &= ~((TIM_CCMR1_CC1S | TIM_CCMR1_OC1FE | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M) << shift);
    *CCMR |= (p_mode << 4) << shift;

    switch (p_channel) {
    case e_TimerViaSTM32F4_Channel1:
        this->m_timer.CCR1 = p_pulse;
        break;
    case e_TimerViaSTM32F4_Channel2:
        this->m_timer.CCR2 = p_pulse;
        break;
    case e_TimerViaSTM32F4_Channel3:
        this->m_timer.CCR3 = p_pulse;
        break;
    case e_TimerViaSTM32F4_Channel4:
        this->m_timer.CCR4 = p_pulse;
        break;
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
TimerViaSTM32F4::enableOutputChannel(const TimerChannel_t p_channel, const bool p_activeLow /* = false */) const {
    this->m_timer.CCER &= ~((TIM_CCER_CC1P | TIM_CCER_CC1E) << (p_channel * 4));
    this->m_timer.CCER |= TIM_CCER_CC1E << (p_channel * 4);
    if (p_activeLow) {
        this->m_timer.CCER |= TIM_CCER_CC1P << (p_channel * 4);
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void 
TimerViaSTM32F4::disableOutputChannel(const TimerChannel_t p_channel) const {
    this->m_timer.CCER &= ~(TIM_CCER_CC1E << (p_channel * 4));
}

/*******************************************************************************
 * 
 ******************************************************************************/
void 
TimerViaSTM32F4::enableUpdateIrq() const {
    this->m_timer.DIER |= TIM_DIER_UIE;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void 
TimerViaSTM32F4::disableUpdateIrq() const {
    this->m_timer.DIER &= ~TIM_DIER_UIE;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void 
TimerViaSTM32F4::handleIrq() const {
    uint32_t sr = this->m_timer.SR;
    
    if (sr & TIM_SR_UIF) {
        this->m_timer.SR &= ~TIM_SR_UIF;
    }
}

} /* namespace timer */

#endif /* _TIMER_STM32F4_HPP_227fb640_e182_4eca_9f47_b665bea6ea0b */