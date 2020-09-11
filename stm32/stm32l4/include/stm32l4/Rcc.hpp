/*-
 * $Copyright$
 */
#ifndef _STM32L4_RCC_HPP_CB6747A9_0C67_4230_9C47_FB5F714E7B3F
#define _STM32L4_RCC_HPP_CB6747A9_0C67_4230_9C47_FB5F714E7B3F

#include <stm32f4xx.h>

#include <stm32/Rcc.hpp>
#include <stm32/Gpio.hpp>
#include <stm32l4/Mco.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace l4 {
/*****************************************************************************/

template<
    typename PllCfg,
    typename Flash,
    typename Pwr
>
class RccT
    : public RccViaSTM32T<
        RccT<PllCfg, Flash, Pwr>,
        ::stm32::l4::Mco
    >
{
public:
    typedef enum FunctionAPB1R1_e {
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
        e_USART2    = RCC_APB1ENR1_USART2EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_USART3    = RCC_APB1ENR1_USART3EN,
        e_USART4    = RCC_APB1ENR1_USART4EN,
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
    } FunctionAPB1R1_t;

    /* FIXME These live in a different register; enabling them won't work unless the respective enable() function is created. */
    typedef enum FunctionAPB1R2_e {
        e_LowPwrUart1   = RCC_APB1ENR2_LPUART1EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_I2c4          = RCC_APB1ENR2_I2C4EN,
#endif
        e_Swp           = RCC_APB1ENR2_SWPMI1EN,
        e_LowPwrTim2    = RCC_APB1ENR2_LPTIM2EN,
    } FunctionAPB1R2_t;

    typedef enum FunctionAPB2_e {
        e_SysCfg    = RCC_APB2ENR_SYSCFGEN,
        e_Fw        = RCC_APB2ENR_FWEN,
        e_Tim1      = RCC_APB2ENR_TIM1EN,
        e_Spi1      = RCC_APB2ENR_SPI1EN,
        e_USART1    = RCC_APB2ENR_USART1EN,
        e_Tim15     = RCC_APB2ENR_TIM15EN,
        e_Tim16     = RCC_APB2ENR_TIM16EN,
        e_Sai1      = RCC_APB2ENR_SAI1EN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_Dfsdm1    = RCC_APB2ENR_DFSDM1EN,
#endif
    } FunctionAPB2_t;

    typedef enum FunctionAHB1_e {
#if defined(RCC_AHB1ENR_DMA1EN)
        e_Dma1          = RCC_AHB1ENR_DMA1EN,
#endif
#if defined(RCC_AHB1ENR_DMA2EN)
        e_Dma2          = RCC_AHB1ENR_DMA2EN,
#endif
#if defined(RCC_AHB1ENR_FLASHEN)
        e_Flash         = RCC_AHB1ENR_FLASHEN,
#endif
#if defined(RCC_AHB1ENR_CRCEN)
        e_Crc           = RCC_AHB1ENR_CRCEN,
#endif
#if defined(RCC_AHB1ENR_TSCEN)
        e_Tsc           = RCC_AHB1ENR_TSCEN,
#endif
    } FunctionAHB1_t;

    typedef enum FunctionAHB2_e {
        e_GPIOA         = RCC_AHB2ENR_GPIOAEN,
        e_GPIOB         = RCC_AHB2ENR_GPIOBEN,
        e_GPIOC         = RCC_AHB2ENR_GPIOCEN,
/* FIXME Does the H/W really not exist or is it just the #define that is missing? */
#if 0 
        e_GPIOD         = RCC_AHB2ENR_GPIODEN,
        e_GPIOE         = RCC_AHB2ENR_GPIOEEN,
#endif
        e_GPIOH         = RCC_AHB2ENR_GPIOHEN,
        e_Adc           = RCC_AHB2ENR_ADCEN,
        e_Rng           = RCC_AHB2ENR_RNGEN,
    } FunctionAHB2_t;

    typedef enum FunctionAHB3_e {
#if defined(RCC_AHB3ENR_QSPIEN)
        e_QuadSPI       = RCC_AHB3ENR_QSPIEN,
#endif
    } FunctionAHB3_t;

    RccT(RCC_TypeDef * const p_rcc, const PllCfg &p_pllCfg, const Flash &p_flash, const Pwr &p_pwr)
        : RccViaSTM32T< RccT< PllCfg, Flash, Pwr >, ::stm32::l4::Mco>(*p_rcc), m_pllCfg(p_pllCfg) {
        this->setupSafeMode();

        this->setupPLL(p_pllCfg);

        this->enableHSE(m_pllCfg.enableHSE());
        this->enablePLL(m_pllCfg.enablePLL());
        this->enableMSI(m_pllCfg.enableMSI());

        this->setupFlash(p_flash);
        p_pwr.setupVoltageScaling();

        this->switchSysclk(p_pllCfg.getSysclkSource());

        /* Turn off HSI, if not required */
        this->enableHSI(p_pllCfg.enableHSI());
    }

    void setupPLL(const PllCfg &p_pllCfg) const {
        this->m_rcc.PLLCFGR = p_pllCfg.getPllCfgReg();

        /*
        * Set up dividers for AHB, APB1 (low-speed peripheral) and APB2 (high
        * speed peripheral) busses.
        */
        this->m_rcc.CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);

        this->m_rcc.CFGR |= (p_pllCfg.m_ahbPrescaler << RCC_CFGR_HPRE_Pos) & RCC_CFGR_HPRE_Msk;
        this->m_rcc.CFGR |= (p_pllCfg.m_apb1Prescaler << RCC_CFGR_PPRE1_Pos) & RCC_CFGR_PPRE1_Msk;
        this->m_rcc.CFGR |= (p_pllCfg.m_apb2Prescaler << RCC_CFGR_PPRE2_Pos) & RCC_CFGR_PPRE2_Msk;
    }

    void enable(const FunctionAPB1R1_t p_function) const {
        this->m_rcc.APB1ENR1 |= p_function;
    }

    void disable(const FunctionAPB1R1_t p_function) const {
        this->m_rcc.APB1ENR1 &= ~p_function;
    }

    void enable(const FunctionAPB2_t p_function) const {
        this->m_rcc.APB2ENR |= p_function;
    }

    void disable(const FunctionAPB2_t p_function) const {
        this->m_rcc.APB2ENR &= ~p_function;
    }

    void enable(const FunctionAHB1_t p_function) const {
        this->m_rcc.AHB1ENR |= p_function;
    }

    void disable(const FunctionAHB1_t p_function) const {
        this->m_rcc.AHB1ENR &= ~p_function;
    }

    void enable(const FunctionAHB2_t p_function) const {
        this->m_rcc.AHB2ENR |= p_function;
    }

    void disable(const FunctionAHB2_t p_function) const {
        this->m_rcc.AHB2ENR &= ~p_function;
    }

    void enable(const FunctionAHB3_t p_function) const;
    void disable(const FunctionAHB3_t p_function) const;

    unsigned getClockSpeed(const FunctionAPB1R1_t /* p_function */) const {
        return this->m_pllCfg.getApb1SpeedInHz();
    }

    unsigned getClockSpeed(const FunctionAPB2_t /* p_function */) const {
        return this->m_pllCfg.getApb2SpeedInHz();
    }

private:
    const PllCfg &  m_pllCfg;

    void enableMSI(const bool p_enable) const {
        this->m_rcc.CR &= ~(RCC_CR_MSION | RCC_CR_MSIRGSEL);

        if (p_enable) {
            this->m_rcc.CR &= ~RCC_CR_MSIRANGE_Msk;
            this->m_rcc.CR |= (this->m_pllCfg.m_msiRange << RCC_CR_MSIRANGE_Pos) & RCC_CR_MSIRANGE_Msk;
            this->m_rcc.CR |= (RCC_CR_MSION | RCC_CR_MSIPLLEN | RCC_CR_MSIRGSEL);

            while (!(this->m_rcc.CR & RCC_CR_MSIRDY));
        }
    }

    void setupSafeMode(void) const {
        /* Enable HSI and wait until it's ready */
        this->m_rcc.CR |= RCC_CR_HSION;
        while (!(this->m_rcc.CR & RCC_CR_HSIRDY)) ;

        /* Set up the internal oscillator as the system clock */
        this->m_rcc.CFGR &= ~RCC_CFGR_SW;
        while (this->m_rcc.CFGR & RCC_CFGR_SWS);

        /* Disable external Oscillator, the clock security system and the internal PLL */
        this->m_rcc.CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON /* | RCC_CR_MSION */);
    }

    void setupFlash(const Flash &p_flash) const {
        p_flash.enableICache(true);
        p_flash.enableDCache(true);
        p_flash.enablePrefetch(true);
    }
};

/*****************************************************************************/
    } /* namespace l4 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32L4_RCC_HPP_CB6747A9_0C67_4230_9C47_FB5F714E7B3F */
