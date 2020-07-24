/*-
 * $Copyright$
 */
#ifndef _STM32F1_SCP_HPP_1A752CA0_F283_440B_9FB6_C15D4A0D8C58
#define _STM32F1_SCP_HPP_1A752CA0_F283_440B_9FB6_C15D4A0D8C58

#include <stm32/Cpu.hpp>

namespace stm32 {
    namespace f1 {
        class Scb {
            SCB_Type &  m_scb;
        public:
            constexpr Scb(SCB_Type * const p_scb) : m_scb(*p_scb) {

            }
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_SCP_HPP_1A752CA0_F283_440B_9FB6_C15D4A0D8C58 */
