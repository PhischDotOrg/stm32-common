/*-
 * $Copyright$
 */
#ifndef _RCC_HPP_156F7387_0A65_4E1C_804D_549AA087FE78
#define _RCC_HPP_156F7387_0A65_4E1C_804D_549AA087FE78

#include <stm32f4xx.h>
#include <stm32/Cpu.hpp>
#include <stm32/RccEngine.hpp>

#include <type_traits>

namespace stm32 {
    template<
        typename RccT,
        typename McoPolicyT
    >
    class RccViaSTM32T : public McoPolicyT {
    public:
        template<typename MCOPinT>
        void
        setMCO(const MCOPinT &p_mcoPin, const typename McoPolicyT::MCO1Output_t p_output, const typename McoPolicyT::MCOPrescaler_t p_prescaler) const {
            McoPolicyT::setMCO(this->m_rcc, p_mcoPin, p_output, p_prescaler);
        }

        template<typename EngineT>
        void
        enableEngine(const EngineT & /* p_engine */) const {
            // static_assert( std::is_null_pointer< decltype(BusTypeT<EngineT>::m_busType) >::value != false, "Engine Type is not mapped to RCC Type");
            static_cast<const RccT *>(this)->enable(BusTypeT<EngineT>::m_busType);
        }

        template<typename EngineT>
        void
        disableEngine(const EngineT & /* p_obj */) const {
            // static_assert( std::is_null_pointer< decltype(BusTypeT<EngineT>::m_busType) >::value != false, "Engine Type is not mapped to RCC Type");
            static_cast<const RccT *>(this)->disable(BusTypeT<EngineT>::m_busType);
        }

        template<typename EngineT>
        unsigned
        getEngineClockSpeed(const EngineT & /* p_obj */) const {
            // static_assert( std::is_null_pointer< decltype(BusTypeT<EngineT>::m_busType) >::value != false, "Engine Type is not mapped to RCC Type");
            return static_cast<const RccT *>(this)->getClockSpeed(BusTypeT<EngineT>::m_busType);
        }

    protected:
        RCC_TypeDef &   m_rcc;

        void setupSafeMode(void) const {
            /* Enable HSI and wait until it's ready */
            m_rcc.CR |= RCC_CR_HSION;
            while (!(m_rcc.CR & RCC_CR_HSIRDY)) ;

            /* Set up the internal oscillator as the system clock */
            m_rcc.CFGR &= ~RCC_CFGR_SW;
            while (m_rcc.CFGR & RCC_CFGR_SWS);

            /* Disable external Oscillator, the clock security system and the internal PLL */
            m_rcc.CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON /* | RCC_CR_MSION */);
        }

        void enableHSE(const bool p_enable) const {
            m_rcc.CR &= ~RCC_CR_HSEON_Msk;

            /*
            * If enable is requested, then enable external oscillator input and wait
            * for signal to settle.
            */
            if (p_enable) {
                m_rcc.CR |= RCC_CR_HSEON;
                while (!(m_rcc.CR & RCC_CR_HSERDY_Msk));
            }
        }

        void enableHSI(const bool p_enable) const {
            if (!p_enable) {
                m_rcc.CR &= ~RCC_CR_HSION_Msk;
            } else {
                while ((m_rcc.CR & RCC_CR_HSION_Msk) != RCC_CR_HSION_Msk) ;
                m_rcc.CR |= RCC_CR_HSION;
            }
        }

        void enablePLL(const bool p_enable) const {
            m_rcc.CR &= ~RCC_CR_PLLON;
            
            if (p_enable) {
                m_rcc.CR |= RCC_CR_PLLON;
                while (!(m_rcc.CR & RCC_CR_PLLRDY));
            }
        }

        template<typename SysclkSourceT>
        void switchSysclk(const SysclkSourceT &p_sysclkSrc) {
            m_rcc.CFGR &= ~RCC_CFGR_SW_Msk;
            m_rcc.CFGR |= (p_sysclkSrc << RCC_CFGR_SW_Pos) & RCC_CFGR_SW_Msk;

            while ((m_rcc.CFGR & RCC_CFGR_SWS) != ((p_sysclkSrc << RCC_CFGR_SWS_Pos) & RCC_CFGR_SWS_Msk));
        }

        constexpr RccViaSTM32T(RCC_TypeDef & p_rcc) : m_rcc(p_rcc) {
            /* Make protected so only sub-classes can instantiate */
        }

        ~RccViaSTM32T() {
            /* Make protected to avoid virtual declaration for Destrutor */
        }
    }; /* class Rcc */
} /* namespace stm32 */

#endif /* _RCC_HPP_156F7387_0A65_4E1C_804D_549AA087FE78 */
