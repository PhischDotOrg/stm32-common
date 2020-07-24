/*-
 * $Copyright$
 */
#ifndef _STM32F1_RCC_HPP_58BC1A38_BDBD_4E15_BC81_DBFB690A0A7A
#define _STM32F1_RCC_HPP_58BC1A38_BDBD_4E15_BC81_DBFB690A0A7A

#include <stm32f4xx.h>

#include <stm32/Rcc.hpp>

#include <stm32f1/Mco.hpp>
#include <stm32f1/PllCfg.hpp>

namespace stm32 {
    namespace f1 {
        class Rcc : public RccViaSTM32T<Rcc, McoPolicy> {
            const PllCfg &  m_pllCfg;

        public:
            typedef enum FunctionAPB1_e {
                e_USART2        = RCC_APB1ENR_USART2EN,
                e_USART3        = RCC_APB1ENR_USART3EN,
            } FunctionAPB1_t;

            typedef enum FunctionAPB2_e {
                e_USART1        = RCC_APB2ENR_USART1EN,
                e_GPIOA         = RCC_APB2ENR_IOPAEN,
                e_GPIOB         = RCC_APB2ENR_IOPBEN,
                e_GPIOC         = RCC_APB2ENR_IOPCEN,
                e_GPIOD         = RCC_APB2ENR_IOPDEN,
                e_GPIOE         = RCC_APB2ENR_IOPEEN,
            } FunctionAPB2_t;

            typedef enum FunctionAHB1_e {
            } FunctionAHB1_t;

            Rcc(RCC_TypeDef * const p_rcc, const PllCfg &p_pllCfg, const Flash &p_flash, const Pwr & /* p_pwr */)
              : RccViaSTM32T(*p_rcc), m_pllCfg(p_pllCfg) /* , m_flash(p_flash), m_pwr(p_pwr) */ {
                p_flash.setupLatency(p_pllCfg.getSysclkSpeedInHz());

                enableHSE(p_pllCfg.enableHSE());

                setupPLL(p_pllCfg);
                enablePLL(p_pllCfg.enablePLL());

                switchSysclk(p_pllCfg.getSysclkSource());

                /* Turn off HSI, if not required */
                enableHSI(p_pllCfg.enableHSI());
            }

            void setupPLL(const PllCfg &m_pllCfg) const {
                m_rcc.CFGR &= ~RCC_CFGR_PLLMULL_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_pllM << RCC_CFGR_PLLMULL_Pos) & RCC_CFGR_PLLMULL_Msk;

                m_rcc.CFGR &= ~RCC_CFGR_PLLXTPRE_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_hsePrescaler << RCC_CFGR_PLLXTPRE_Pos) & RCC_CFGR_PLLXTPRE_Msk;

                m_rcc.CFGR &= ~RCC_CFGR_PLLSRC_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_pllSource << RCC_CFGR_PLLSRC_Pos) & RCC_CFGR_PLLSRC_Msk;

                m_rcc.CFGR &= ~RCC_CFGR_PPRE2_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_apb2Prescaler << RCC_CFGR_PPRE2_Pos) & RCC_CFGR_PPRE2_Msk;

                m_rcc.CFGR &= ~RCC_CFGR_PPRE1_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_apb1Prescaler << RCC_CFGR_PPRE1_Pos) & RCC_CFGR_PPRE1_Msk;

                m_rcc.CFGR &= ~RCC_CFGR_HPRE_Msk;
                m_rcc.CFGR |= (m_pllCfg.m_ahbPrescaler << RCC_CFGR_HPRE_Pos) & RCC_CFGR_HPRE_Msk;
            }

            void enable(const FunctionAPB1_e &p_engine) const {
                this->m_rcc.APB1ENR |= p_engine;
            }

            void disable(const FunctionAPB1_e &p_engine) const {
                this->m_rcc.APB1ENR |= ~p_engine;
            }

            void enable(const FunctionAPB2_e &p_engine) const {
                this->m_rcc.APB2ENR |= p_engine;
            }

            void disable(const FunctionAPB2_e &p_engine) const {
                this->m_rcc.APB2ENR |= ~p_engine;
            }

            unsigned getClockSpeed(const FunctionAPB1_t /* p_function */) const {
                return this->m_pllCfg.getApb1SpeedInHz();
            }

            unsigned getClockSpeed(const FunctionAPB2_t /* p_function */) const {
                return this->m_pllCfg.getApb2SpeedInHz();
            }
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_RCC_HPP_58BC1A38_BDBD_4E15_BC81_DBFB690A0A7A */
