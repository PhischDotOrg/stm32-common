/*-
 * $Copyright$
 */
#ifndef _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B
#define _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B

#include <stm32/Cpu.hpp>

namespace stm32 {
    namespace f1 {
        class Pwr {
            PWR_TypeDef & m_pwr;

        public:
            constexpr Pwr(PWR_TypeDef * const p_pwr) : m_pwr(*p_pwr) {

            }
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B */
