/*-
 * $Copyright$
 */
#ifndef _SCP_HPP_42FA0EBE_1F0E_4D24_ADD7_F3D6AE708A90
#define _SCP_HPP_42FA0EBE_1F0E_4D24_ADD7_F3D6AE708A90

#include <stm32/Cpu.hpp>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
class Scb {
private:
    SCB_Type &  m_scb;

    /* Must write this key to Bits[31:16] of AIRCR for write to take effect */
    static constexpr uint32_t m_aircr_key = 0x05FAu << 16;

    void writeAIRCR(const uint16_t p_value) const {
        m_scb.AIRCR = m_aircr_key | p_value;
    }

    void updateAIRCR(const uint16_t p_value, const uint16_t p_mask) const {
        uint16_t aircr = m_scb.AIRCR & 0xFFFF;

        aircr &= ~p_mask;
        aircr |= p_value;
        
        writeAIRCR(aircr);
    }

public:
    constexpr Scb(SCB_Type * const p_scb) : m_scb(*p_scb) {

    }

    typedef enum ScbPriorityGroup_e {
        e_PriorityGroup_0 = 0x7,
        e_PriorityGroup_1 = 0x6,
        e_PriorityGroup_2 = 0x5,
        e_PriorityGroup_3 = 0x4,
        e_PriorityGroup_4 = 0x3,
    } ScbPriorityGroup_t;
    
    void configurePriorityGroup(const ScbPriorityGroup_t p_priorityGroup) const {
        this->updateAIRCR(p_priorityGroup << SCB_AIRCR_PRIGROUP_Pos, SCB_AIRCR_PRIGROUP_Msk);
    }

    ScbPriorityGroup_t getPriorityGroup(void) const {
        return static_cast<ScbPriorityGroup_t>(((m_scb.AIRCR & SCB_AIRCR_PRIGROUP_Msk) >> SCB_AIRCR_PRIGROUP_Pos));
    }
};
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _SCP_HPP_42FA0EBE_1F0E_4D24_ADD7_F3D6AE708A90 */
