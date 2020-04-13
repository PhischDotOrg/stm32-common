/*-
 * $Copyright$
-*/

#ifndef _SCB_STM32F4_HPP_7e2dd6a3_204c_4e57_8953_3835019aa1df
#define _SCB_STM32F4_HPP_7e2dd6a3_204c_4e57_8953_3835019aa1df

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class ScbViaSTM32F4 {
private:
    SCB_Type * const    m_scb;

    static const uint32_t m_aircr_key;

    void writeAIRCR(const uint16_t p_value) const;
    void updateAIRCR(const uint16_t p_value, const uint16_t p_mask) const;

public:
    ScbViaSTM32F4(SCB_Type * const p_scb);
    ~ScbViaSTM32F4(void);

    typedef enum ScbPriorityGroup_e {
        e_PriorityGroup_0 = 0x7,
        e_PriorityGroup_1 = 0x6,
        e_PriorityGroup_2 = 0x5,
        e_PriorityGroup_3 = 0x4,
        e_PriorityGroup_4 = 0x3,
    } ScbPriorityGroup_t;
    
    void configurePriorityGroup(const ScbPriorityGroup_t p_priorityGroup) const;
    ScbPriorityGroup_t getPriorityGroup(void) const;
}; /* class ScbViaSTM32F4 */

} /* devices */

#endif /* _SCB_STM32F4_HPP_7e2dd6a3_204c_4e57_8953_3835019aa1df */
