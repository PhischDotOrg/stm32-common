/*-
 * $Copyright$
-*/

#ifndef _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5
#define _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5

#include <assert.h>

#include "RccViaSTM32.hpp"
#include "FlashViaSTM32F4.hpp"
#include "PwrViaSTM32F4.hpp"

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class Stm32F407xx {
private:
        Stm32F407xx(void);
public:
    typedef enum FunctionAPB1_e {
        e_Tim2      = RCC_APB1ENR_TIM2EN,
        e_Tim3      = RCC_APB1ENR_TIM3EN,
        e_Tim4      = RCC_APB1ENR_TIM4EN,
        e_Tim5      = RCC_APB1ENR_TIM5EN,
#if defined(RCC_APB1ENR_TIM6EN)
        e_Tim6      = RCC_APB1ENR_TIM6EN,
#endif
#if defined(RCC_APB1ENR_TIM7EN)
        e_Tim7      = RCC_APB1ENR_TIM7EN,
#endif
#if defined(RCC_APB1ENR_TIM12EN)
        e_Tim12     = RCC_APB1ENR_TIM12EN,
#endif
#if defined(RCC_APB1ENR_TIM13EN)        
        e_Tim13     = RCC_APB1ENR_TIM13EN,
#endif
#if defined(RCC_APB1ENR_TIM14EN)       
        e_Tim14     = RCC_APB1ENR_TIM14EN,
#endif
        e_Wwdg      = RCC_APB1ENR_WWDGEN,
        e_Spi2      = RCC_APB1ENR_SPI2EN,
        e_Spi3      = RCC_APB1ENR_SPI3EN,
        e_Usart2    = RCC_APB1ENR_USART2EN,
#if defined(RCC_APB1ENR_USART3EN)
        e_Usart3    = RCC_APB1ENR_USART3EN,
#endif
#if defined(RCC_APB1ENR_UART4EN)
        e_Uart4     = RCC_APB1ENR_UART4EN,
#endif
#if defined(RCC_APB1ENR_UART5EN)
        e_Uart5     = RCC_APB1ENR_UART5EN,
#endif
        e_I2c1      = RCC_APB1ENR_I2C1EN,
        e_I2c2      = RCC_APB1ENR_I2C2EN,
        e_I2c3      = RCC_APB1ENR_I2C3EN,
#if defined(RCC_APB1ENR_CAN1EN)
        e_Can1      = RCC_APB1ENR_CAN1EN,
#endif
#if defined(RCC_APB1ENR_CAN2EN)
        e_Can2      = RCC_APB1ENR_CAN2EN,
#endif
        e_Pwr       = RCC_APB1ENR_PWREN,
#if defined(RCC_APB1ENR_DACEN)
        e_Dac       = RCC_APB1ENR_DACEN,
#endif
    } FunctionAPB1_t;

    typedef enum FunctionAPB2_e {
        e_Tim1      = RCC_APB2ENR_TIM1EN,
#if defined(RCC_APB2ENR_TIM8EN)
        e_Tim8      = RCC_APB2ENR_TIM8EN,
#endif
        e_Usart1    = RCC_APB2ENR_USART1EN,
        e_Usart6    = RCC_APB2ENR_USART6EN,
        e_Adc1      = RCC_APB2ENR_ADC1EN,
#if defined(RCC_APB2ENR_ADC2EN)
        e_Adc2      = RCC_APB2ENR_ADC2EN,
#endif
#if defined(RCC_APB2ENR_ADC3EN)
        e_Adc3      = RCC_APB2ENR_ADC3EN,
#endif
        e_Sdio      = RCC_APB2ENR_SDIOEN,
        e_Spi1      = RCC_APB2ENR_SPI1EN,
        e_SysCfg    = RCC_APB2ENR_SYSCFGEN,
        e_Tim9      = RCC_APB2ENR_TIM9EN,
        e_Tim10     = RCC_APB2ENR_TIM10EN,
        e_Tim11     = RCC_APB2ENR_TIM11EN,
    } FunctionAPB2_t;

    typedef enum FunctionAHB1_e {
        e_GpioA         = RCC_AHB1ENR_GPIOAEN,
        e_GpioB         = RCC_AHB1ENR_GPIOBEN,
        e_GpioC         = RCC_AHB1ENR_GPIOCEN,
        e_GpioD         = RCC_AHB1ENR_GPIODEN,
        e_GpioE         = RCC_AHB1ENR_GPIOEEN,
#if defined(RCC_AHB1ENR_GPIOFEN)
        e_GpioF         = RCC_AHB1ENR_GPIOFEN,
#endif
#if defined(RCC_AHB1ENR_GPIOGEN)
        e_GpioG         = RCC_AHB1ENR_GPIOGEN,
#endif
        e_GpioH         = RCC_AHB1ENR_GPIOHEN,
#if defined(RCC_AHB1ENR_GPIOIEN)
        e_GpioI         = RCC_AHB1ENR_GPIOIEN,
#endif
        e_Crc           = RCC_AHB1ENR_CRCEN,
#if defined(RCC_AHB1ENR_BKPSRAMEN)
        e_BackupSram    = RCC_AHB1ENR_BKPSRAMEN,
#endif
#if defined(RCC_AHB1ENR_CCMDATARAMEN)
        e_CcmDataRam    = RCC_AHB1ENR_CCMDATARAMEN,
#endif
        e_Dma1          = RCC_AHB1ENR_DMA1EN,
        e_Dma2          = RCC_AHB1ENR_DMA2EN,
#if defined(RCC_AHB1ENR_ETHMACEN)
        e_EthMac        = RCC_AHB1ENR_ETHMACEN,
#endif
#if defined(RCC_AHB1ENR_ETHMACTXEN)
        e_EthMacTx      = RCC_AHB1ENR_ETHMACTXEN,
#endif
#if defined(RCC_AHB1ENR_ETHMACRXEN)
        e_EthMacRx      = RCC_AHB1ENR_ETHMACRXEN,
#endif
#if defined(RCC_AHB1ENR_ETHMACPTPEN)
        e_EthMacPtp     = RCC_AHB1ENR_ETHMACPTPEN,
#endif
#if defined(RCC_AHB1ENR_OTGHSEN)
        e_OtgHs         = RCC_AHB1ENR_OTGHSEN,
#endif
#if defined(RCC_AHB1ENR_OTGHSULPIEN)
        e_OtgHsUlpi     = RCC_AHB1ENR_OTGHSULPIEN,
#endif
    } FunctionAHB1_t;

    typedef enum FunctionAHB2_e {
#if defined(RCC_AHB2ENR_DCMIEN)
        e_DCMIEN    = RCC_AHB2ENR_DCMIEN,
#endif
#if defined(RCC_AHB2ENR_CRYPEN)
        e_CRYPEN    = RCC_AHB2ENR_CRYPEN,
#endif
#if defined(RCC_AHB2ENR_HASHEN)
        e_HASHEN    = RCC_AHB2ENR_HASHEN,
#endif
#if defined(RCC_AHB2ENR_RNGEN)
        e_RNGEN     = RCC_AHB2ENR_RNGEN,
#endif
#if defined(RCC_AHB2ENR_OTGFSEN)
        e_OtgFs     = RCC_AHB2ENR_OTGFSEN,
#endif
    } FunctionAHB2_t;

    typedef enum FunctionAHB3_e {
#if defined(RCC_AHB3ENR_FSMCEN)
        e_FSMCEN    = RCC_AHB3ENR_FSMCEN,
#endif
    } FunctionAHB3_t;

    typedef enum {
        e_SysclkHSI = 0,
        e_SysclkHSE = 1,
        e_SysclkPLL = 2,
        e_SysclkInvalid = 3,
    } SysclkSource_t;

    typedef enum {
        e_PllSourceHSI = 0,
        e_PllSourceHSE = 1,
    } PllSource_t;

    typedef enum {
        e_PllP_Div2 = 0,
        e_PllP_Div4 = 1,
        e_PllP_Div6 = 2,
        e_PllP_Div8 = 3,
    } PllP_t;

    typedef enum {
        e_PllQ_Div2     = 2,
        e_PllQ_Div3     = 3,
        e_PllQ_Div4     = 4,
        e_PllQ_Div5     = 5,
        e_PllQ_Div6     = 6,
        e_PllQ_Div7     = 7,
        e_PllQ_Div8     = 8,
        e_PllQ_Div9     = 9,
        e_PllQ_Div10    = 10,
        e_PllQ_Div11    = 11,
        e_PllQ_Div12    = 12,
        e_PllQ_Div13    = 13,
        e_PllQ_Div14    = 14,
        e_PllQ_Div15    = 15,
        e_PllQ_Disabled
    } PllQ_t;

    typedef enum {
        e_AHBPrescaler_Div2     = 0,
        e_AHBPrescaler_Div4     = 8,
        e_AHBPrescaler_Div8     = 9,
        e_AHBPrescaler_Div16    = 10,
        e_AHBPrescaler_Div64    = 11,
        e_AHBPrescaler_Div128   = 12,
        e_AHBPrescaler_Div256   = 13,
        e_AHBPrescaler_Div512   = 14,
        e_AHBPrescaler_None     = 15
    } AHBPrescaler_t;

    typedef enum {
        e_APBPrescaler_None     = 0,
        e_APBPrescaler_Div2     = 4,
        e_APBPrescaler_Div4     = 5,
        e_APBPrescaler_Div8     = 6,
        e_APBPrescaler_Div16    = 7,
    } APBPrescaler_t;
};

/*******************************************************************************
 *
 ******************************************************************************/
template<typename Stm32FxxCpuT = Stm32F407xx>
struct PllConfigurationValuesT {
/*******************************************************************************
 * Types 
 ******************************************************************************/
    typedef Stm32FxxCpuT Stm32FxxCpu_t;

    typedef typename Stm32FxxCpuT::PllSource_t      PllSource_t;
    typedef typename Stm32FxxCpuT::SysclkSource_t   SysclkSource_t;

    typedef typename Stm32FxxCpuT::AHBPrescaler_t   AHBPrescaler_t;
    typedef typename Stm32FxxCpuT::APBPrescaler_t   APBPrescaler_t;

    typedef typename Stm32FxxCpuT::PllP_t           PllP_t;
    typedef typename Stm32FxxCpuT::PllQ_t           PllQ_t;

/*******************************************************************************
 * Values
 ******************************************************************************/
    const PllSource_t       m_pllSource;

    const unsigned          m_hseSpeedInHz;

    const unsigned          m_pllM;

    const unsigned          m_pllN;
    const PllP_t            m_pllP;
    const PllQ_t            m_pllQ;

    const SysclkSource_t    m_sysclkSource;
    const AHBPrescaler_t    m_ahbPrescaler;
    const APBPrescaler_t    m_apb1Prescaler;
    const APBPrescaler_t    m_apb2Prescaler;
};

typedef struct PllConfigurationValuesT<> PllConfigurationValues;

template<typename PllConfigurationValuesT /* = PllConfigurationValuesT<> */ >
class PllConfigurationT {
public:
    typedef typename PllConfigurationValuesT::Stm32FxxCpu_t     Stm32FxxCpu_t;

    typedef typename PllConfigurationValuesT::PllSource_t       PllSource_t;
    typedef typename PllConfigurationValuesT::SysclkSource_t    SysclkSource_t;

    typedef typename PllConfigurationValuesT::AHBPrescaler_t    AHBPrescaler_t;
    typedef typename PllConfigurationValuesT::APBPrescaler_t    APBPrescaler_t;

    typedef typename PllConfigurationValuesT::PllP_t            PllP_t;
    typedef typename PllConfigurationValuesT::PllQ_t            PllQ_t;

private:
    const PllConfigurationValuesT & m_pllCfg;
    const unsigned                  m_hseSpeedInHz;
    static const unsigned           m_hsiSpeedInHz = 16 * 1000 * 1000;
    const unsigned                  m_sysclkSpeedInHz;
    const uint32_t                  m_pllCfgReg;
    const bool                      m_enableHSE;
    const bool                      m_enableHSI;
    const bool                      m_enablePLL;

    constexpr unsigned getHSESpeedInHz(void) const {
        return m_pllCfg.m_hseSpeedInHz;
    }

    constexpr unsigned getHSISpeedInHz(void) const {
        static_assert(m_hsiSpeedInHz == 16 * 1000 * 1000);

        return m_hsiSpeedInHz;
    }

    constexpr unsigned getPllInputSpeedInHz(const PllConfigurationValuesT &p_pllCfg) const {
        return (
            (p_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSI) ? getHSISpeedInHz()
              : (p_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSE) ? getHSESpeedInHz()
                : 0
        );
    }

    constexpr unsigned getPllSpeedInHz(void) const {
        unsigned vco = getPllVcoSpeedInHz();

        unsigned pllp = 2 << m_pllCfg.m_pllP;

        return (vco + (pllp / 2)) / pllp;
    }

    constexpr unsigned getPllVcoSpeedInHz(void) const {
        const unsigned input = getPllInputSpeedInHz(this->m_pllCfg);

        const unsigned plln = m_pllCfg.m_pllN;
        const unsigned pllm = m_pllCfg.m_pllM;

        const unsigned vco = ((input * plln) + (pllm / 2)) / pllm;

        return vco;
    }

    constexpr unsigned getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const {
        const unsigned value = (
            APBPrescaler_t::e_APBPrescaler_None   == p_prescaler ? 1
          : APBPrescaler_t::e_APBPrescaler_Div2   == p_prescaler ? 2
          : APBPrescaler_t::e_APBPrescaler_Div4   == p_prescaler ? 4
          : APBPrescaler_t::e_APBPrescaler_Div8   == p_prescaler ? 8
          : APBPrescaler_t::e_APBPrescaler_Div16  == p_prescaler ? 16
          : 0
        );

        return value;        
    }
    
    static constexpr unsigned getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) {
        const unsigned value = (
            AHBPrescaler_t::e_AHBPrescaler_None   == p_prescaler ? 1
          : AHBPrescaler_t::e_AHBPrescaler_Div2   == p_prescaler ? 2
          : AHBPrescaler_t::e_AHBPrescaler_Div4   == p_prescaler ? 4
          : AHBPrescaler_t::e_AHBPrescaler_Div8   == p_prescaler ? 8
          : AHBPrescaler_t::e_AHBPrescaler_Div16  == p_prescaler ? 16
          : AHBPrescaler_t::e_AHBPrescaler_Div64  == p_prescaler ? 64
          : AHBPrescaler_t::e_AHBPrescaler_Div128 == p_prescaler ? 128
          : AHBPrescaler_t::e_AHBPrescaler_Div256 == p_prescaler ? 256
          : AHBPrescaler_t::e_AHBPrescaler_Div512 == p_prescaler ? 512
          : 0
        );

        return value;        
    }

protected:
    constexpr unsigned getSysclkSpeedInHz(void) const {
        const SysclkSource_t sysclkSrc = getSysclkSource();

        const unsigned speed =
             sysclkSrc == SysclkSource_t::e_SysclkHSI ? getHSISpeedInHz()
              : sysclkSrc == SysclkSource_t::e_SysclkHSE ? getHSESpeedInHz()
                : sysclkSrc == SysclkSource_t::e_SysclkPLL ? getPllSpeedInHz()
                  : 0;

        return speed;
    }

    constexpr unsigned getAhbSpeedInHz(void) const {
        const unsigned prescaler = getAHBPrescalerValue(getAHBPrescaler());

        const unsigned ahbSpeedInHz = (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;

        return ahbSpeedInHz;
    }

    constexpr unsigned getApb1SpeedInHz(void) const {
        const unsigned prescaler = getAPBPrescalerValue(getAPB1Prescaler());

        return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
    }

    constexpr unsigned getApb2SpeedInHz(void) const {
        unsigned prescaler = getAPBPrescalerValue(getAPB2Prescaler());

        return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;    
    }

    constexpr bool enableHSE(void) const {
        return (m_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSE) || (m_pllCfg.m_sysclkSource == SysclkSource_t::e_SysclkHSE);
    }

    constexpr bool enablePLL(void) const {
        return (m_pllCfg.m_sysclkSource == SysclkSource_t::e_SysclkPLL);
    }

    constexpr bool enableHSI(void) const {
        return (m_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSI) || (m_pllCfg.m_sysclkSource == SysclkSource_t::e_SysclkHSI);
    }

    constexpr uint32_t getPllCfgReg(void) const {
        const uint32_t pllCfgReg = (
                ((m_pllCfg.m_pllQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk)
              | ((m_pllCfg.m_pllP << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk)
              | ((m_pllCfg.m_pllN <<  RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk)
              | ((m_pllCfg.m_pllM <<  RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk)
              | ((m_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSE) ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI)
        );

        return pllCfgReg;
    }

    constexpr AHBPrescaler_t getAHBPrescaler(void) const {
        return m_pllCfg.m_ahbPrescaler;
    }

    constexpr APBPrescaler_t getAPB1Prescaler(void) const {
        return m_pllCfg.m_apb1Prescaler;
    }

    constexpr APBPrescaler_t getAPB2Prescaler(void) const {
        return m_pllCfg.m_apb2Prescaler;
    }

    constexpr SysclkSource_t getSysclkSource(void) const {
        return m_pllCfg.m_sysclkSource;
    }

public:
    constexpr PllConfigurationT(const PllConfigurationValuesT &p_pllCfg)
      : m_pllCfg(p_pllCfg),
        m_hseSpeedInHz(getHSESpeedInHz()),
        m_sysclkSpeedInHz(getSysclkSpeedInHz()),
        m_pllCfgReg(getPllCfgReg()),
        m_enableHSE(enableHSE()),
        m_enableHSI(enableHSI()),
        m_enablePLL(enablePLL())
    {
    }

#if defined(STM32F407xx)
    static constexpr bool isValid(const PllConfigurationT &p_obj) {
        const bool isValid = true
            && (p_obj.getPllInputSpeedInHz(p_obj.m_pllCfg) >= 4 * 1000 * 1000)
            && (p_obj.getPllInputSpeedInHz(p_obj.m_pllCfg) <= 48 * 1000 * 1000)
            && (p_obj.m_pllCfg.m_pllN > 1) && (p_obj.m_pllCfg.m_pllN < 433)
            && (p_obj.m_pllCfg.m_pllM > 1) && (p_obj.m_pllCfg.m_pllM < 64)
            && (p_obj.getPllVcoSpeedInHz() >= 100 * 1000 * 1000)
            && (p_obj.getPllVcoSpeedInHz() <= 432 * 1000 * 1000)
            && ((p_obj.m_hseSpeedInHz >= 4 * 1000 * 1000) && (p_obj.m_hseSpeedInHz <= 48 * 1000 * 1000))
            && (p_obj.getSysclkSpeedInHz() > 0)
            && (p_obj.getSysclkSpeedInHz() <= 168 * 1000 * 1000)
            && (p_obj.getAhbSpeedInHz() > 0)
            && (p_obj.getAhbSpeedInHz() >= p_obj.getSysclkSpeedInHz())
            && true;

        return isValid;
    }
#endif

#if defined(STM32F411xE)
    static constexpr bool isValid(const PllConfigurationT &p_obj) {
        const bool isValid = true
            && (p_obj.getPllInputSpeedInHz(p_obj.m_pllCfg) >= 4 * 1000 * 1000) && (p_obj.getPllInputSpeedInHz(p_obj.m_pllCfg) <= 48 * 1000 * 1000)
            && (p_obj.m_pllCfg.m_pllN >= 40) && (p_obj.m_pllCfg.m_pllN <= 432)
            && (p_obj.m_pllCfg.m_pllM >= 8) && (p_obj.m_pllCfg.m_pllM <= 63)
            && (p_obj.getPllVcoSpeedInHz() >= 100 * 1000 * 1000) && (p_obj.getPllVcoSpeedInHz() <= 432 * 1000 * 1000)
            && ((p_obj.m_hseSpeedInHz == 0) || ((p_obj.m_hseSpeedInHz >= 1 * 1000 * 1000) && (p_obj.m_hseSpeedInHz <= 50 * 1000 * 1000)))
            && (p_obj.getSysclkSpeedInHz() > 0) && (p_obj.getSysclkSpeedInHz() <= 100 * 1000 * 1000)
            && (p_obj.getAhbSpeedInHz() > 0) && (p_obj.getAhbSpeedInHz() >= p_obj.getSysclkSpeedInHz())
            && (p_obj.getApb1SpeedInHz() > 0) && (p_obj.getApb1SpeedInHz() <= 50 * 1000 * 1000)
            && (p_obj.getApb2SpeedInHz() > 0) && (p_obj.getApb2SpeedInHz() <= 100 * 1000 * 1000)
            && true;

        return isValid;
    }
#endif

};

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename PllConfigurationT>
class RccViaSTM32xxT : public RccViaSTM32Interface<PllConfigurationT> {
public:
    typedef class RccViaSTM32Interface<PllConfigurationT> Parent_t;

    typedef typename Parent_t::Stm32FxxCpu_t Stm32FxxCpu_t;

    typedef typename Stm32FxxCpu_t::FunctionAHB1_e   FunctionAHB1_t;
    typedef typename Stm32FxxCpu_t::FunctionAHB2_e   FunctionAHB2_t;
    typedef typename Stm32FxxCpu_t::FunctionAHB3_e   FunctionAHB3_t;

    typedef typename Stm32FxxCpu_t::FunctionAPB1_e  FunctionAPB1_t;

    typedef typename Stm32FxxCpu_t::FunctionAPB2_e   FunctionAPB2_t;

    typedef typename Stm32FxxCpu_t::AHBPrescaler_t   AHBPrescaler_t;
    typedef typename Stm32FxxCpu_t::APBPrescaler_t   APBPrescaler_t;

    typedef typename Stm32FxxCpu_t::SysclkSource_t  SysclkSource_t;
    typedef typename Stm32FxxCpu_t::PllSource_t     PllSource_t;

public:
    constexpr RccViaSTM32xxT(RCC_TypeDef * const p_rcc, const PllConfigurationT &p_pllCfg, FlashViaSTM32F4 &p_flash, PwrViaSTM32F4 &p_pwr)
      : RccViaSTM32Interface<PllConfigurationT>(p_rcc, p_pllCfg, p_flash, p_pwr) {
        this->setupSafeMode();

        this->enableHSE(p_pllCfg.enableHSE());
        this->enablePLL(p_pllCfg.enablePLL());

        /*
        * Set up dividers for AHB, APB1 (low-speed peripheral) and APB2 (high
        * speed peripheral) busses.
        */
        this->m_rcc->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
        if (this->m_pllCfg.getAHBPrescaler() != AHBPrescaler_t::e_AHBPrescaler_None) {
            this->m_rcc->CFGR |= (this->m_pllCfg.getAHBPrescaler() << RCC_CFGR_HPRE_Pos) & RCC_CFGR_HPRE_Msk;
        }
        
        if (this->m_pllCfg.getAPB1Prescaler() != APBPrescaler_t::e_APBPrescaler_None) {
            this->m_rcc->CFGR |= (this->m_pllCfg.getAPB1Prescaler() << RCC_CFGR_PPRE1_Pos) & RCC_CFGR_PPRE1_Msk;
        }

        if (this->m_pllCfg.getAPB2Prescaler() != APBPrescaler_t::e_APBPrescaler_None) {
            this->m_rcc->CFGR |= (this->m_pllCfg.getAPB2Prescaler() << RCC_CFGR_PPRE2_Pos) & RCC_CFGR_PPRE2_Msk;
        }

        /* Finally, switch the clock source to whatever was requested */
        this->m_rcc->CFGR |= (this->m_pllCfg.getSysclkSource() << RCC_CFGR_SW_Pos) & RCC_CFGR_SW_Msk;
        while ((this->m_rcc->CFGR & RCC_CFGR_SWS) != ((this->m_pllCfg.getSysclkSource() << RCC_CFGR_SWS_Pos) & RCC_CFGR_SWS_Msk));

        this->setupFlash();

        this->setupPower();

        /* Turn off HSI, if not required */
        this->enableHSI(this->m_pllCfg.enableHSI());
    }

    ~RccViaSTM32xxT(void) {};

    typedef enum {
        e_MCOPre_None   = 0,
        e_MCOPre_2      = 4,
        e_MCOPre_3      = 5,
        e_MCOPre_4      = 6,
        e_MCOPre_5      = 7,
        e_MCOInvalid
    } MCOPrescaler_t;

    typedef enum class MCO1Output_e : uint8_t {
        e_HSI    = 0,
        e_LSE    = 1,
        e_HSE    = 2,
        e_PLL    = 3,
        e_None
    } MCO1Output_t;

    typedef enum class MCO2Output_e {
        e_Sysclk = 0,
        e_PllI2S = 1,
        e_HSE    = 2,
        e_PLL    = 3,
        e_None
    } MCO2Output_t;

    template<typename MCOPinT, typename GpioAccessT>
    void setMCO(const MCOPinT &p_mcoPin, const MCO1Output_t p_output, const MCOPrescaler_t p_prescaler) {
        this->m_rcc->CFGR &= ~RCC_CFGR_MCO1PRE_Msk;
        this->m_rcc->CFGR |= (p_prescaler << RCC_CFGR_MCO1PRE_Pos) & RCC_CFGR_MCO1PRE_Msk;

        this->m_rcc->CFGR &= ~RCC_CFGR_MCO1_Msk;
        this->m_rcc->CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCO1_Pos) & RCC_CFGR_MCO1_Msk;

        if (p_output != MCO1Output_t::e_None) {
            p_mcoPin.enable(GpioAccessT::e_Alternate, GpioAccessT::e_None, GpioAccessT::e_Mco);
        } else {
            p_mcoPin.enable(GpioAccessT::e_Input, GpioAccessT::e_None, GpioAccessT::e_Gpio);
        }
    }

    template<typename MCOPinT, typename GpioAccessT>
    void setMCO(const MCOPinT &p_mcoPin, const MCO2Output_t p_output, const MCOPrescaler_t p_prescaler) {
        this->m_rcc->CFGR &= ~RCC_CFGR_MCO2PRE_Msk;
        this->m_rcc->CFGR |= (p_prescaler << RCC_CFGR_MCO2PRE_Pos) & RCC_CFGR_MCO2PRE_Msk;

        this->m_rcc->CFGR &= ~RCC_CFGR_MCO2_Msk;
        this->m_rcc->CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCO2_Pos) & RCC_CFGR_MCO2_Msk;

        if (p_output != MCO2Output_t::e_None) {
            p_mcoPin.enable(GpioAccessT::e_Alternate, GpioAccessT::e_None, GpioAccessT::e_Mco);
        } else {
            p_mcoPin.enable(GpioAccessT::e_Input, GpioAccessT::e_None, GpioAccessT::e_Gpio);
        }
    }

    void enable(const FunctionAPB1_t p_function) const {
        this->m_rcc->APB1ENR |= p_function;
    }

    void disable(const FunctionAPB1_t p_function) const {
        this->m_rcc->APB1ENR &= ~p_function;
    }

    void enable(const FunctionAPB2_t p_function) const {
        this->m_rcc->APB2ENR |= p_function;
    }
   
    void disable(const FunctionAPB2_t p_function) const {
        this->m_rcc->APB2ENR &= ~p_function;
    }

    void enable(const FunctionAHB1_t p_function) const {
        this->m_rcc->AHB1ENR |= p_function;
    }

    void disable(const FunctionAHB1_t p_function) const {
        this->m_rcc->AHB1ENR &= ~p_function;
    }

    void enable(const FunctionAHB2_t p_function) const {
        this->m_rcc->AHB2ENR |= p_function;
    }

    void disable(const FunctionAHB2_t p_function) const {
        this->m_rcc->AHB2ENR &= ~p_function;
    }

    void enable(const FunctionAHB3_t p_function) const;
    void disable(const FunctionAHB3_t p_function) const;

    unsigned getClockSpeed(const FunctionAPB1_t /* p_function */) const {
        return this->m_pllCfg.getApb1SpeedInHz();
    }

    unsigned getClockSpeed(const FunctionAPB2_t /* p_function */) const {
        return this->m_pllCfg.getApb2SpeedInHz();
    }
}; /* class RccViaSTM32F4 */

/*******************************************************************************
 * 
 ******************************************************************************/
typedef RccViaSTM32xxT< const PllConfigurationInterfaceT< const struct PllConfigurationValuesT< Stm32F407xx> > > RccViaSTM32F4;

} /* namespace devices */

#endif /* _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5 */
