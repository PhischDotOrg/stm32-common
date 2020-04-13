/*-
 * $Copyright$
-*/

#ifndef _SCB_STM32F4_CPP_dad0b8b6_c97c_4ad5_a34e_7d6daed59394
#define _SCB_STM32F4_CPP_dad0b8b6_c97c_4ad5_a34e_7d6daed59394

#include <stm32f4/ScbViaSTM32F4.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace devices {

/* Must write this key to Bits[31:16] of AIRCR for write to take effect */
const uint32_t ScbViaSTM32F4::m_aircr_key = 0x05FAu << 16;

/*******************************************************************************
 * 
 ******************************************************************************/
ScbViaSTM32F4::ScbViaSTM32F4(SCB_Type * const p_scb) : m_scb(p_scb) {
    
}

/*******************************************************************************
 * 
 ******************************************************************************/
ScbViaSTM32F4::~ScbViaSTM32F4() {
    
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
ScbViaSTM32F4::writeAIRCR(const uint16_t p_value) const {
    this->m_scb->AIRCR = m_aircr_key | p_value;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
ScbViaSTM32F4::updateAIRCR(const uint16_t p_value, const uint16_t p_mask) const {
    uint16_t aircr = this->m_scb->AIRCR & 0xFFFF;

    aircr &= ~p_mask;
    aircr |= p_value;
    
    this->writeAIRCR(aircr);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
ScbViaSTM32F4::configurePriorityGroup(const ScbPriorityGroup_t p_priorityGroup) const {
    this->updateAIRCR(p_priorityGroup << SCB_AIRCR_PRIGROUP_Pos, SCB_AIRCR_PRIGROUP_Msk);
}

/*******************************************************************************
 * 
 ******************************************************************************/
ScbViaSTM32F4::ScbPriorityGroup_t
ScbViaSTM32F4::getPriorityGroup() const {
    return static_cast<ScbPriorityGroup_t>(((this->m_scb->AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
}

} /* namespace devices */

#endif /* _SCB_STM32F4_CPP_dad0b8b6_c97c_4ad5_a34e_7d6daed59394 */
