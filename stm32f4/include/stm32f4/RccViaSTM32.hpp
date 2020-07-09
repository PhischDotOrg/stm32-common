/*-
 * $Copyright$
 */

#ifndef _RCC_STM32_HPP_C56D1C11_8813_4247_81D0_63FFA4DAA081
#define _RCC_STM32_HPP_C56D1C11_8813_4247_81D0_63FFA4DAA081

/*******************************************************************************
 * 
 ******************************************************************************/
#include <cstdint>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

    #include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include "FlashViaSTM32F4.hpp"
#include "PwrViaSTM32F4.hpp"

/*******************************************************************************
 * 
 ******************************************************************************/
namespace devices {

template<typename PllConfigurationValuesT> class PllConfigurationT;

template<typename PllConfigurationValuesT>
class PllConfigurationInterfaceT : public PllConfigurationT<PllConfigurationValuesT> {
public:
    typedef class PllConfigurationT<PllConfigurationValuesT> Parent_t;

    typedef typename Parent_t::Stm32FxxCpu_t Stm32FxxCpu_t;

    constexpr PllConfigurationInterfaceT(const PllConfigurationValuesT &p_pllCfg)
      : PllConfigurationT<PllConfigurationValuesT>(p_pllCfg) {

    }

    constexpr unsigned getSysclkSpeedInHz(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getSysclkSpeedInHz();
    }

    constexpr unsigned getApb1SpeedInHz(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getApb1SpeedInHz();
    }

    constexpr unsigned getApb2SpeedInHz(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getApb2SpeedInHz();
    }

    constexpr unsigned getAhbSpeedInHz(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getAhbSpeedInHz();
    }

    constexpr bool enableHSE(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::enableHSE();
    }

    constexpr bool enablePLL(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::enablePLL();
    }

    constexpr bool enableHSI(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::enableHSI();
    }

    constexpr uint32_t getPllCfgReg(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getPllCfgReg();
    }

    constexpr typename Parent_t::AHBPrescaler_t getAHBPrescaler(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getAHBPrescaler();
    }

    constexpr typename Parent_t::APBPrescaler_t getAPB1Prescaler(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getAPB1Prescaler();
    }

    constexpr typename Parent_t::APBPrescaler_t getAPB2Prescaler(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getAPB2Prescaler();
    }

    constexpr typename Parent_t::SysclkSource_t getSysclkSource(void) const {
        return PllConfigurationT<PllConfigurationValuesT>::getSysclkSource();
    }
};

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename PllConfigurationT>
class RccViaSTM32Interface {
protected:
    RCC_TypeDef * const         m_rcc;
    const PllConfigurationT &   m_pllCfg;
    FlashViaSTM32F4 &           m_flash;
    PwrViaSTM32F4 &             m_pwr;

    void setupSafeMode(void) const {
        /* Enable HSI and wait until it's ready */
        this->m_rcc->CR |= RCC_CR_HSION;
        while (!(this->m_rcc->CR & RCC_CR_HSIRDY)) ;

        /* Set up the internal oscillator as the system clock */
        this->m_rcc->CFGR &= ~RCC_CFGR_SW;
        while (this->m_rcc->CFGR & RCC_CFGR_SWS);

        /* Disable external Oscillator, the clock security system and the internal PLL */
        this->m_rcc->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON /* | RCC_CR_MSION */);
    }

    void enableHSE(const bool p_enable) const {
        this->m_rcc->CR &= ~RCC_CR_HSEON;

        /*
        * If enable is requested, then enable external oscillator input and wait
        * for signal to settle.
        */
        if (p_enable) {
            this->m_rcc->CR |= RCC_CR_HSEON;
            while (!(this->m_rcc->CR & RCC_CR_HSERDY));
        }
    }

    void enableHSI(const bool p_enable) const {
        this->m_rcc->CR &= ~RCC_CR_HSION;

        /*
        * If enable is requested, then enable the internal oscillator and wait for
        * it to become ready.
        */
        if (p_enable) {
            this->m_rcc->CR |= RCC_CR_HSION;
            while (!(this->m_rcc->CR & RCC_CR_HSIRDY));
        }
    }

    void enablePLL(const bool p_enable) const {
        this->m_rcc->CR &= ~RCC_CR_PLLON;
        this->m_rcc->PLLCFGR = 0x00001000;
        
        if (p_enable) {
            const uint32_t pllCfgReg = this->m_pllCfg.getPllCfgReg();
            // static_assert(pllCfgReg != 0); FIXME

            this->m_rcc->PLLCFGR = pllCfgReg;

            this->m_rcc->CR |= RCC_CR_PLLON;
            while (!(this->m_rcc->CR & RCC_CR_PLLRDY));
        }
    }

    void setupFlash(void) const {
        m_flash.enableICache(true);
        m_flash.enableDCache(true);
        m_flash.enablePrefetch(true);

        unsigned targetSpeed = this->m_pllCfg.getSysclkSpeedInHz();
        
        /* FIXME Assuming 3V Operation for now */
        if (targetSpeed <= 30000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_0WaitStates);
        } else if (targetSpeed <= 64000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_1WaitStates);
        } else if (targetSpeed <= 90000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_2WaitStates);
        } else if (targetSpeed <= 120000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_3WaitStates);        
        } else if (targetSpeed <= 150000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_4WaitStates);        
        } else if (targetSpeed <= 168000000u) {
            m_flash.setupLatency(FlashViaSTM32F4::e_FlashLatency_5WaitStates);        
        } else {
            while (1);
        }
    }

    void setupPower(void) const {
        unsigned targetSpeed = this->m_pllCfg.getSysclkSpeedInHz();

        m_pwr.setupVoltageScaling(targetSpeed);
    }

public:
    typedef typename PllConfigurationT::Stm32FxxCpu_t Stm32FxxCpu_t;

    constexpr RccViaSTM32Interface(RCC_TypeDef * const p_rcc, const PllConfigurationT &p_pllCfg, FlashViaSTM32F4 &p_flash, PwrViaSTM32F4 &p_pwr)
      : m_rcc(p_rcc), m_pllCfg(p_pllCfg), m_flash(p_flash), m_pwr(p_pwr) {

    }
};

} /* namespace devices */


/*******************************************************************************
 * 
 ******************************************************************************/
#if defined(STM32L432xx)
    #include "RccViaSTM32L4.hpp"
#endif /* defined(STM32L432xx) */

#if defined(STM32F407xx) \
 || defined(STM32F411xE)
    #include "RccViaSTM32F4.hpp"
#endif /* defined(STM32F407xx) */

#endif /* _RCC_STM32_HPP_C56D1C11_8813_4247_81D0_63FFA4DAA081 */
