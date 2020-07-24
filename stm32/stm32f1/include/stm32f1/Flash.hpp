/*-
 * $Copyright$
 */
#ifndef _STM32F1_FLASH_HPP_86E31F7B_70FC_46DA_B3E4_F8FDFC00F99B
#define _STM32F1_FLASH_HPP_86E31F7B_70FC_46DA_B3E4_F8FDFC00F99B

namespace stm32 {
    namespace f1 {
        class Flash {
            FLASH_TypeDef &m_flash;

            enum FlashLatency_e {
                e_FlashLatency_ZeroWait = 0,
                e_FlashLatency_OneWait  = 1,
                e_FlashLatency_TwoWait  = 2,
            };

        public:
            constexpr Flash(FLASH_TypeDef * const p_flash) : m_flash(*p_flash) {

            }

            ~Flash() {

            }

            void setupLatency(unsigned p_sysclkSpeedInHz) const {
                m_flash.ACR &= ~FLASH_ACR_LATENCY_Msk;

                if (p_sysclkSpeedInHz <= 24 * 1000 * 1000) {
                    m_flash.ACR |= (e_FlashLatency_ZeroWait << FLASH_ACR_LATENCY_Pos) & FLASH_ACR_LATENCY_Msk;
                } else if (p_sysclkSpeedInHz <= 48 * 1000 * 1000) {
                    m_flash.ACR |= (e_FlashLatency_OneWait << FLASH_ACR_LATENCY_Pos) & FLASH_ACR_LATENCY_Msk;
                } else /* if (p_sysclkSpeedInHz <= 72 * 1000 * 1000) */ {
                    m_flash.ACR |= (e_FlashLatency_TwoWait << FLASH_ACR_LATENCY_Pos) & FLASH_ACR_LATENCY_Msk;
                }
            }
        };
    }
}

#endif /* _STM32F1_FLASH_HPP_86E31F7B_70FC_46DA_B3E4_F8FDFC00F99B */
