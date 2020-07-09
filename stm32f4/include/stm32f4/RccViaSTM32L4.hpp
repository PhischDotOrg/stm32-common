/*-
 * $Copyright$
-*/

#ifndef _RCC_STM32L4_HPP_00A295BE_33C9_4AB9_8C03_7D7B498F60E1
#define _RCC_STM32L4_HPP_00A295BE_33C9_4AB9_8C03_7D7B498F60E1

#include "RccViaSTM32.hpp"

/*******************************************************************************
 * 
 ******************************************************************************/
namespace devices {

class Stm32L4xx {
private:
        Stm32L4xx(void);
public:
    enum FunctionAHB1_e {
        e_Dma1          = RCC_AHB1ENR_DMA1EN,
        e_Dma2          = RCC_AHB1ENR_DMA2EN,
        e_Flash         = RCC_AHB1ENR_FLASHEN,
        e_Crc           = RCC_AHB1ENR_CRCEN,
        e_Tsc           = RCC_AHB1ENR_TSCEN,
    };

    enum FunctionAHB2_e {
        e_GpioA         = RCC_AHB2ENR_GPIOAEN,
        e_GpioB         = RCC_AHB2ENR_GPIOBEN,
        e_GpioC         = RCC_AHB2ENR_GPIOCEN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_GpioD         = RCC_AHB2ENR_GPIODEN,
        e_GpioE         = RCC_AHB2ENR_GPIOEEN,
#endif
        e_GpioH         = RCC_AHB2ENR_GPIOHEN,
        e_Adc           = RCC_AHB2ENR_ADCEN,
        e_Rng           = RCC_AHB2ENR_RNGEN,
    };

    enum FunctionAHB3_e {
        e_QuadSPI       = RCC_AHB3ENR_QSPIEN,
    };

    enum FunctionAPB1R1_e {
        e_Tim2      = RCC_APB1ENR1_TIM2EN,
        e_Tim6      = RCC_APB1ENR1_TIM6EN,
        e_Tim7      = RCC_APB1ENR1_TIM7EN,
        e_Rtc       = RCC_APB1ENR1_RTCAPBEN,
        e_Wwdg      = RCC_APB1ENR1_WWDGEN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_Spi2      = RCC_APB1ENR1_SPI2EN,
#endif
        e_Spi3      = RCC_APB1ENR1_SPI3EN,
        e_Usart2    = RCC_APB1ENR1_USART2EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_Usart3    = RCC_APB1ENR1_USART3EN,
        e_Usart4    = RCC_APB1ENR1_USART4EN,
#endif
        e_I2c1      = RCC_APB1ENR1_I2C1EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_I2c2      = RCC_APB1ENR1_I2C2EN,
#endif
        e_I2c3      = RCC_APB1ENR1_I2C3EN,
        e_Crs       = RCC_APB1ENR1_CRSEN,
        e_Can1      = RCC_APB1ENR1_CAN1EN,
        e_UsbFs     = RCC_APB1ENR1_USBFSEN,
        e_Pwr       = RCC_APB1ENR1_PWREN,
        e_Dac1      = RCC_APB1ENR1_DAC1EN,
        e_OpAmp     = RCC_APB1ENR1_OPAMPEN,
        e_LowPwrTim1 = RCC_APB1ENR1_LPTIM1EN,
    };

    /* FIXME These live in a different register; enabling them won't work unless the respective enable() function is created. */
    enum FunctionAPB1R2_e {
        e_LowPwrUart1   = RCC_APB1ENR2_LPUART1EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_I2c4          = RCC_APB1ENR2_I2C4EN,
#endif
        e_Swp           = RCC_APB1ENR2_SWPMI1EN,
        e_LowPwrTim2    = RCC_APB1ENR2_LPTIM2EN,
    };

    enum FunctionAPB2_e {
        e_SysCfg    = RCC_APB2ENR_SYSCFGEN,
        e_Fw        = RCC_APB2ENR_FWEN,
        e_Tim1      = RCC_APB2ENR_TIM1EN,
        e_Spi1      = RCC_APB2ENR_SPI1EN,
        e_Usart1    = RCC_APB2ENR_USART1EN,
        e_Tim15     = RCC_APB2ENR_TIM15EN,
        e_Tim16     = RCC_APB2ENR_TIM16EN,
        e_Sai1      = RCC_APB2ENR_SAI1EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_Dfsdm1    = RCC_APB2ENR_DFSDM1EN,
#endif
    };

    typedef enum {
        e_SysclkMSI = 0,
        e_SysclkHSI = 1,
        e_SysclkHSE = 2,
        e_SysclkPLL = 3,
        e_SysclkInvalid,
    } SysclkSource_t;

    typedef enum {
        e_PllSourceNone = 0,
        e_PllSourceMSI  = 1,
        e_PllSourceHSI  = 2,
        e_PllSourceHSE  = 3,
    } PllSource_t;

    typedef enum {
        e_PllR_Div2 = 0,
        e_PllR_Div4 = 1,
        e_PllR_Div6 = 2,
        e_PllR_Div8 = 3,
        e_PllR_Disabled
    } PllR_t;

    typedef enum {
        e_PllQ_Div2 = 0,
        e_PllQ_Div4 = 1,
        e_PllQ_Div6 = 2,
        e_PllQ_Div8 = 3,
        e_PllQ_Disabled
    } PllQ_t;

    typedef enum {
        e_PllP_Div7     = 0,
        e_PllP_Div17    = 1,
        e_PllP_Disabled
    } PllP_t;

    typedef enum {
        e_AHBPrescaler_Div2     = 0,
        e_AHBPrescaler_Div4     = 1,
        e_AHBPrescaler_Div8     = 2,
        e_AHBPrescaler_Div16    = 3,
        e_AHBPrescaler_Div64    = 4,
        e_AHBPrescaler_Div128   = 5,
        e_AHBPrescaler_Div256   = 6,
        e_AHBPrescaler_Div512   = 7,
        e_AHBPrescaler_None     = 8,
    } AHBPrescaler_t;

    typedef enum {
        e_APBPrescaler_Div2     = 0,
        e_APBPrescaler_Div4     = 1,
        e_APBPrescaler_Div8     = 2,
        e_APBPrescaler_Div16    = 3,
        e_APBPrescaler_None     = 4,
    } APBPrescaler_t;

    typedef enum {
        e_MSIRange_100kHz       = 0,
        e_MSIRange_200kHz       = 1,
        e_MSIRange_400kHz       = 2,
        e_MSIRange_800kHz       = 3,
        e_MSIRange_1MHz         = 4,
        e_MSIRange_2MHz         = 5,
        e_MSIRange_4MHz         = 6,
        e_MSIRange_8MHz         = 7,
        e_MSIRange_16MHz        = 8,
        e_MSIRange_24MHz        = 9,
        e_MSIRange_32MHz        = 10,
        e_MSIRange_48MHz        = 11,
        e_MSIRange_Invalid
    } MSIRange_t;
};

/*******************************************************************************
 *
 ******************************************************************************/
template<typename Stm32FxxCpuT>
struct PllConfigurationValuesT {
public:
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
    typedef typename Stm32FxxCpuT::PllR_t           PllR_t;

    typedef typename Stm32FxxCpuT::MSIRange_t       MSIRange_t;

/*******************************************************************************
 * Values
 ******************************************************************************/
    const PllSource_t       m_pllSource;

    const MSIRange_t        m_msiRange;
    const unsigned          m_hseSpeedInHz;

    const unsigned          m_pllM;

    const unsigned          m_pllN;
    const PllP_t            m_pllP;
    const PllQ_t            m_pllQ;
    const PllR_t            m_pllR;

    const unsigned          m_pllSaiN;
    const PllP_t            m_pllSaiP;
    const PllQ_t            m_pllSaiQ;
    const PllR_t            m_pllSaiR;

    const SysclkSource_t    m_sysclkSource;
    const AHBPrescaler_t    m_ahbPrescaler;
    const APBPrescaler_t    m_apb1Prescaler;
    const APBPrescaler_t    m_apb2Prescaler;
};

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename PllConfigurationValuesT>
class PllConfigurationT {
public:
    typedef typename PllConfigurationValuesT::Stm32FxxCpu_t     Stm32FxxCpu_t;

    typedef typename PllConfigurationValuesT::PllSource_t       PllSource_t;
    typedef typename PllConfigurationValuesT::SysclkSource_t    SysclkSource_t;

    typedef typename PllConfigurationValuesT::AHBPrescaler_t    AHBPrescaler_t;
    typedef typename PllConfigurationValuesT::APBPrescaler_t    APBPrescaler_t;

    typedef typename PllConfigurationValuesT::PllP_t            PllP_t;
    typedef typename PllConfigurationValuesT::PllR_t            PllR_t;
    typedef typename PllConfigurationValuesT::PllQ_t            PllQ_t;

    typedef typename PllConfigurationValuesT::MSIRange_t        MSIRange_t;

private:
    const PllConfigurationValuesT & m_pllCfg;
    const unsigned                  m_msiSpeedInHz;
    static const unsigned           m_hsiSpeedInHz = 16 * 1000 * 1000;
    const unsigned                  m_sysclkSpeedInHz;
    const uint32_t                  m_pllCfgReg;
    const bool                      m_enableHSE;
    const bool                      m_enableHSI;
    const bool                      m_enablePLL;
    const bool                      m_enableMSI;

    constexpr unsigned getMSISpeedInHz(void) const {
        unsigned msiSpeedInHz =
            m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_100kHz ?   100 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_200kHz ?   200 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_400kHz ?   100 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_800kHz ?   200 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_1MHz   ?  1000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_2MHz   ?  2000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_4MHz   ?  4000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_8MHz   ?  8000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_16MHz  ? 16000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_24MHz  ? 24000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_32MHz  ? 32000 * 1000
          : m_pllCfg.m_msiRange == MSIRange_t::e_MSIRange_48MHz  ? 48000 * 1000
          : 0;

        return msiSpeedInHz;
    }

    constexpr unsigned getHSESpeedInHz(void) const {
        return m_pllCfg.m_hseSpeedInHz;
    }

    constexpr unsigned getHSISpeedInHz(void) const {
        static_assert(m_hsiSpeedInHz == 16 * 1000 * 1000);

        return m_hsiSpeedInHz;
    }

    constexpr unsigned getPllInputSpeedInHz(void) const {
        return (
          (m_pllCfg.m_pllSource == PllSource_t::e_PllSourceMSI) ? getMSISpeedInHz()
            : (m_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSI) ? getHSISpeedInHz()
              : (m_pllCfg.m_pllSource == PllSource_t::e_PllSourceHSE) ? getHSESpeedInHz()
                : 0
        );

        // FIXME
        // static_assert(speed > 0);
        // static_assert(speed <= 48 * 1000 * 1000);

        // return speed;
    }

    constexpr unsigned getPllSpeedInHz(void) const {
        const unsigned vco = getPllVcoSpeedInHz();

        const unsigned pllr = 2 << m_pllCfg.m_pllR;
     
        return (vco + (pllr / 2)) / pllr;
    }

    constexpr unsigned getPllVcoSpeedInHz(void) const {
        const unsigned input = getPllInputSpeedInHz();
        // static_assert(input != 0);

        const unsigned plln = m_pllCfg.m_pllN;
        // static_assert((plln > 8) && (plln <= 86));

        const unsigned pllm = m_pllCfg.m_pllM;
        // static_assert(pllm > 0);

        const unsigned vco = ((input * plln) + (pllm / 2)) / pllm;
        // static_assert(vco != 0);

        return vco;
    }

    constexpr unsigned getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const {
        const unsigned value = (p_prescaler == APBPrescaler_t::e_APBPrescaler_None ? 1 : 2 << p_prescaler);
        // static_assert(value != 0);

        return value;        
    }

    constexpr unsigned getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) const {
        unsigned value = 0;

        switch (p_prescaler) {
        case AHBPrescaler_t::e_AHBPrescaler_Div2:
        case AHBPrescaler_t::e_AHBPrescaler_Div4:
        case AHBPrescaler_t::e_AHBPrescaler_Div8:
        case AHBPrescaler_t::e_AHBPrescaler_Div16:
            value = 2 << p_prescaler;
            break;
        case AHBPrescaler_t::e_AHBPrescaler_Div64:
        case AHBPrescaler_t::e_AHBPrescaler_Div128:
        case AHBPrescaler_t::e_AHBPrescaler_Div256:
        case AHBPrescaler_t::e_AHBPrescaler_Div512:
            value = 4 << p_prescaler;
            break;
        case AHBPrescaler_t::e_AHBPrescaler_None:
            value = 1;
            break;
        }

        return value;        
    }

    constexpr bool getPllPEnable() const {
        const bool enable = m_pllCfg.m_pllP != PllP_t::e_PllP_Disabled;
        // static_assert(enable == false); /* FIXME */

        return (enable);
    }

    constexpr bool getPllQEnable() const {
        bool enable = (m_pllCfg.m_pllQ != PllQ_t::e_PllQ_Disabled);
        // static_assert(enable == true); /* FIXME */

        return (enable);
    }

    constexpr bool getPllREnable() const {
        bool enable = (m_pllCfg.m_pllR != PllR_t::e_PllR_Disabled);
        // static_assert(enable == true); /* FIXME */

        return (enable);
    }

    constexpr uint32_t getPllSaiCfgReg(void) const {
        constexpr uint32_t pllSaiCfgReg = (
                  ((m_pllCfg.m_pllSaiN << RCC_PLLSAI1CFGR_PLLSAI1N_Pos) & RCC_PLLSAI1CFGR_PLLSAI1N_Msk)

                | ((m_pllCfg.m_pllP << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk)
                | ((getPllPEnable() << RCC_PLLCFGR_PLLPEN_Pos) & RCC_PLLCFGR_PLLPEN_Msk)

                | ((m_pllCfg.m_pllQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk)
                | ((getPllQEnable() << RCC_PLLCFGR_PLLQEN_Pos) & RCC_PLLCFGR_PLLQEN_Msk)

                | ((m_pllCfg.m_pllR << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR_Msk)
                | ((getPllREnable() << RCC_PLLCFGR_PLLREN_Pos) & RCC_PLLCFGR_PLLREN_Msk)

                | ((m_pllCfg.m_pllSource << RCC_PLLCFGR_PLLSRC_Pos) & RCC_PLLCFGR_PLLSRC_Msk)
            );

        return pllSaiCfgReg;
    }

protected:
    constexpr unsigned getSysclkSpeedInHz(void) const {
        SysclkSource_t sysclkSrc = getSysclkSource();

        unsigned speed =
            sysclkSrc == SysclkSource_t::e_SysclkMSI ? getMSISpeedInHz()
              : sysclkSrc == SysclkSource_t::e_SysclkHSI ? getHSISpeedInHz()
                : sysclkSrc == SysclkSource_t::e_SysclkHSE ? getHSESpeedInHz()
                  : sysclkSrc == SysclkSource_t::e_SysclkPLL ? getPllSpeedInHz()
                    : 0;

        return speed;
    }

    constexpr unsigned getAhbSpeedInHz(void) const {
        unsigned prescaler = getAHBPrescalerValue(getAHBPrescaler());
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 512);

        unsigned ahbSpeedInHz = (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
        // FIXME
        // static_assert(ahbSpeedInHz > 0);
        // static_assert(ahbSpeedInHz <= 80 * 1000 * 1000);

        return ahbSpeedInHz;
    }

    constexpr unsigned getApb1SpeedInHz(void) const {
        unsigned prescaler = getAPBPrescalerValue(getAPB1Prescaler());
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 16);

        return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
    }

    constexpr unsigned getApb2SpeedInHz(void) const {
        unsigned prescaler = getAPBPrescalerValue(getAPB2Prescaler());
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 16);

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
        uint32_t pllCfgReg = (
                  (((m_pllCfg.m_pllM - 1) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk)
                | ((m_pllCfg.m_pllN << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk)

                | ((m_pllCfg.m_pllP << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk)
                | ((getPllPEnable() << RCC_PLLCFGR_PLLPEN_Pos) & RCC_PLLCFGR_PLLPEN_Msk)

                | ((m_pllCfg.m_pllQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk)
                | ((getPllQEnable() << RCC_PLLCFGR_PLLQEN_Pos) & RCC_PLLCFGR_PLLQEN_Msk)

                | ((m_pllCfg.m_pllR << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR_Msk)
                | ((getPllREnable() << RCC_PLLCFGR_PLLREN_Pos) & RCC_PLLCFGR_PLLREN_Msk)

                | ((m_pllCfg.m_pllSource << RCC_PLLCFGR_PLLSRC_Pos) & RCC_PLLCFGR_PLLSRC_Msk)
            );

        // static_assert(pllCfgReg != 0);

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
        m_msiSpeedInHz(getMSISpeedInHz()),
        m_sysclkSpeedInHz(getSysclkSpeedInHz()),
        m_pllCfgReg(getPllCfgReg()),
        m_enableHSE(enableHSE()),
        m_enableHSI(enableHSI()),
        m_enablePLL(enablePLL()),
        m_enableMSI(enableMSI())
    {
    }

    constexpr bool enableMSI(void) const {
        return ((m_pllCfg.m_pllSource == PllSource_t::e_PllSourceMSI) || (m_pllCfg.m_sysclkSource == SysclkSource_t::e_SysclkMSI));
    }

    constexpr MSIRange_t getMSIRange(void) const {
        return m_pllCfg.m_msiRange;
    }    
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

    typedef typename Stm32FxxCpu_t::FunctionAPB1R1_e FunctionAPB1R1_t;
    typedef typename Stm32FxxCpu_t::FunctionAPB1R2_e FunctionAPB1R2_t;

    typedef typename Stm32FxxCpu_t::FunctionAPB2_e   FunctionAPB2_t;

    typedef typename Stm32FxxCpu_t::AHBPrescaler_t   AHBPrescaler_t;
    typedef typename Stm32FxxCpu_t::APBPrescaler_t   APBPrescaler_t;

    typedef typename Stm32FxxCpu_t::SysclkSource_t  SysclkSource_t;
    typedef typename Stm32FxxCpu_t::PllSource_t     PllSource_t;

private:  
    void enableMSI(const bool p_enable) const {
        this->m_rcc->CR &= ~(RCC_CR_MSION | RCC_CR_MSIRGSEL);

        if (p_enable) {
            this->m_rcc->CR &= ~RCC_CR_MSIRANGE_Msk;
            this->m_rcc->CR |= (this->m_pllCfg.getMSIRange() << RCC_CR_MSIRANGE_Pos) & RCC_CR_MSIRANGE_Msk;
            this->m_rcc->CR |= (RCC_CR_MSION | RCC_CR_MSIPLLEN | RCC_CR_MSIRGSEL);

            while (!(this->m_rcc->CR & RCC_CR_MSIRDY));
        }
    }

public:
    constexpr RccViaSTM32xxT(RCC_TypeDef * const p_rcc, const PllConfigurationT &p_pllCfg, FlashViaSTM32F4 &p_flash, PwrViaSTM32F4 &p_pwr)
      : RccViaSTM32Interface<PllConfigurationT>(p_rcc, p_pllCfg, p_flash, p_pwr) {
        this->setupSafeMode();

        this->enableHSE(this->m_pllCfg.enableHSE());
        this->enablePLL(this->m_pllCfg.enablePLL());
        this->enableMSI(this->m_pllCfg.enableMSI());

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

    typedef enum class MCOPrescaler_e {
        e_MCOPre_1  = 0,
        e_MCOPre_2  = 1,
        e_MCOPre_4  = 2,
        e_MCOPre_8  = 3,
        e_MCOPre_16 = 4,
        e_MCOInvalid
    } MCOPrescaler_t;

    typedef enum class MCOOutput_e {
        e_None    = 0,
        e_Sysclk  = 1,
        e_MSI     = 2,
        e_HSI     = 3,
        e_HSE     = 4,
        e_PLL     = 5,
        e_LSI     = 6,
        e_LSE     = 7,
        e_HSI48   = 8,
    } MCOOutput_t;

    template<typename MCOPinT, typename GpioAccessT>
    void setMCO(const MCOPinT &p_mcoPin, const MCOOutput_t p_output, const MCOPrescaler_t p_prescaler) {
        this->m_rcc->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
        this->m_rcc->CFGR |= (static_cast<unsigned>(p_prescaler) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE_Msk;

        this->m_rcc->CFGR &= ~RCC_CFGR_MCOSEL_Msk;
        this->m_rcc->CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCOSEL_Pos) & RCC_CFGR_MCOSEL_Msk;

        if (p_output != MCOOutput_t::e_None) {
            p_mcoPin.enable(GpioAccessT::e_Alternate, GpioAccessT::e_None, GpioAccessT::e_Mco);
        } else {
            p_mcoPin.enable(GpioAccessT::e_Input, GpioAccessT::e_None, GpioAccessT::e_Gpio);
        }
    }

    void enable(const FunctionAPB1R1_t p_function) const {
        this->m_rcc->APB1ENR[0] |= p_function;
    }

    void disable(const FunctionAPB1R1_t p_function) const {
        this->m_rcc->APB1ENR[0] &= ~p_function;
    }

    void enable(const FunctionAPB1R2_t p_function) const {
        this->m_rcc->APB1ENR[1] |= p_function;
    }

    void disable(const FunctionAPB1R2_t p_function) const {
        this->m_rcc->APB1ENR[1] &= ~p_function;
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

    unsigned getClockSpeed(const FunctionAPB1R1_t /* p_function */) const {
        return this->m_pllCfg.getApb1SpeedInHz();
    }

    unsigned getClockSpeed(const FunctionAPB1R2_t /* p_function */) const {
        return this->m_pllCfg.getApb1SpeedInHz();
    }

    unsigned getClockSpeed(const FunctionAPB2_t /* p_function */) const {
        return this->m_pllCfg.getApb2SpeedInHz();
    }
}; /* class RccViaSTM32F4 */

/*******************************************************************************
 * 
 ******************************************************************************/
typedef RccViaSTM32xxT< const PllConfigurationInterfaceT< const struct PllConfigurationValuesT< Stm32L4xx> > > RccViaSTM32F4;

} /* namespace devices */

#endif /* _RCC_STM32L4_HPP_00A295BE_33C9_4AB9_8C03_7D7B498F60E1 */
