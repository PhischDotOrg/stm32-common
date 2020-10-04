/*-
 * $Copyright$
 */
#ifndef _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B
#define _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B

#include <stm32/Cpu.hpp>

#include <stm32/Engine.hpp>

namespace stm32 {
    namespace f1 {
        template<
            typename RccT,
            typename ScbT,
            intptr_t Address = PWR_BASE
        >
        class PwrT : public EngineT<Address> {
            PWR_TypeDef &   m_pwr;
            const ScbT &    m_scb;

        public:
            enum class Mode_t : uint8_t {
                e_Sleep,
                e_Stop,
                e_Standby
            };

            constexpr PwrT(const RccT &p_rcc, const ScbT &p_scb)
              : m_pwr(* reinterpret_cast<PWR_TypeDef *>(Address)), m_scb(p_scb) {
                p_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));
            }

            void
            enableWkup(void) const {
                m_pwr.CSR |= PWR_CSR_EWUP;
            }

            void
            disableWkup(void) const {
                m_pwr.CSR &= ~PWR_CSR_EWUP_Msk;
            }

            void
            shutdown(Mode_t p_mode) const {
                switch (p_mode) {
                case Mode_t::e_Sleep:
                    /* TODO Sleep Mode not implemented */
                    break;
                case Mode_t::e_Stop:
                    /* TODO Stop Mode not implemented */
                    break;
                case Mode_t::e_Standby:
                    m_scb.setSleepDeep(true);
                    enableWkup();
                    m_pwr.CR |= (PWR_CR_PDDS | PWR_CR_LPDS | PWR_CR_CWUF);
                    while (m_pwr.CSR & PWR_CSR_WUF_Msk);
                    break;
                }

                __asm__ __volatile__("cpsid i");
                __asm__ __volatile__("wfi");
            }
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_PWR_HPP_BB805822_38BA_4B66_8FD5_AD4D2FC5DA5B */
