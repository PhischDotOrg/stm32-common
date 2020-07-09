/*-
 * $Copyright$
-*/

#ifndef _PWR_STM32F4_HPP_56b672c0_6075_4619_a4d1_323dccb5f753
#define _PWR_STM32F4_HPP_56b672c0_6075_4619_a4d1_323dccb5f753

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/FlashViaSTM32F4.hpp>
#include <stm32f4/PwrViaSTM32F4.hpp>
#include <assert.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
PwrViaSTM32F4::PwrViaSTM32F4(PWR_TypeDef* const p_pwr) : m_pwr(p_pwr) {
}

/*******************************************************************************
 * 
 ******************************************************************************/
PwrViaSTM32F4::~PwrViaSTM32F4() {
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
PwrViaSTM32F4::setupVoltageScaling(unsigned /* p_systemSpeed */) const {
#if defined(PWR_CR_VOS)
    m_pwr->CR |= PWR_CR_VOS;
#elif defined(PWR_CR1_VOS)
    m_pwr->CR1 |= PWR_CR1_VOS;
#else
    #error Unhandled else case here!
#endif

}

} /* namespace device */

#endif /* _PWR_STM32F4_HPP_56b672c0_6075_4619_a4d1_323dccb5f753 */