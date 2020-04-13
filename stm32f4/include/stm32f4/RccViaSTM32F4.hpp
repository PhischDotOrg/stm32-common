/*-
 * $Copyright$
-*/

#ifndef _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5
#define _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

extern uint32_t SystemCoreClock;

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <assert.h>

#include "FlashViaSTM32F4.hpp"
#include "PwrViaSTM32F4.hpp"

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class RccViaSTM32F4 {
public:
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

#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    ((uint32_t)25000000) /*!< Default value of the External oscillator in Hz */
#endif /* HSE_VALUE */
    
    class PllConfiguration {
    friend class RccViaSTM32F4;
    private:
        unsigned        m_pllN;
        unsigned        m_pllM;
        PllP_t          m_pllP;
        unsigned        m_pllQ;
        APBPrescaler_t  m_apb1Prescaler;
        APBPrescaler_t  m_apb2Prescaler;
        AHBPrescaler_t  m_ahbPrescaler;
        PllSource_t     m_pllSource;
        SysclkSource_t  m_sysclkSource;
        unsigned        m_hsiSpeedInHz;
        unsigned        m_hseSpeedInHz;

    public:
        PllConfiguration(
            unsigned        p_pllN, unsigned p_pllM,
            PllP_t          p_pllP,
            unsigned        p_pllQ,
            APBPrescaler_t  p_apb1Prescaler = e_APBPrescaler_Div4,
            APBPrescaler_t  p_apb2Prescaler = e_APBPrescaler_Div2,
            AHBPrescaler_t  p_ahbPrescaler = e_AHBPrescaler_None,
            PllSource_t     p_pllSource = e_PllSourceHSI,
            SysclkSource_t  p_sysclkSource = e_SysclkPLL,
            unsigned        p_hsiSpeedInHz = HSI_VALUE,
            unsigned        p_hseSpeedInHz = HSE_VALUE
        ) :
            m_pllN(p_pllN), m_pllM(p_pllM),
            m_pllP(p_pllP),
            m_pllQ(p_pllQ),
            m_apb1Prescaler(p_apb1Prescaler),
            m_apb2Prescaler(p_apb2Prescaler),
            m_ahbPrescaler(p_ahbPrescaler),
            m_pllSource(p_pllSource),
            m_sysclkSource(p_sysclkSource),
            m_hsiSpeedInHz(p_hsiSpeedInHz),
            m_hseSpeedInHz(p_hseSpeedInHz)
        { };
    };

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

    void enable(const FunctionAPB1_t p_function) const;
    void disable(const FunctionAPB1_t p_function) const;

    void enable(const FunctionAPB2_t p_function) const;
    void disable(const FunctionAPB2_t p_function) const;

    void enable(const FunctionAHB1_t p_function) const;
    void disable(const FunctionAHB1_t p_function) const;

    void enable(const FunctionAHB2_t p_function) const;
    void disable(const FunctionAHB2_t p_function) const;

    void enable(const FunctionAHB3_t p_function) const;
    void disable(const FunctionAHB3_t p_function) const;
    
    unsigned getClockSpeed(const FunctionAPB1_t p_function) const;
    unsigned getClockSpeed(const FunctionAPB2_t p_function) const;

    unsigned getAhbSpeedInHz(void) const;
    unsigned getApb1SpeedInHz(void) const;
    unsigned getApb2SpeedInHz(void) const;

    unsigned getSysclkSpeedInHz(void) const;

    RccViaSTM32F4(RCC_TypeDef * const p_rcc, PllConfiguration &p_pllCfg, FlashViaSTM32F4 &p_flash, PwrViaSTM32F4 &p_pwr);
    ~RccViaSTM32F4(void);

private:
    RCC_TypeDef * const m_rcc;
    PllConfiguration &  m_pllCfg;
    FlashViaSTM32F4 &   m_flash;
    PwrViaSTM32F4 &     m_pwr;
    
    void setupSafeMode(void) const;
    void enableHSE(const bool p_enable) const;
    void enableHSI(const bool p_enable) const;
    void enablePLL(const bool p_enable) const;

    SysclkSource_t getSysclkSource(void) const;

    unsigned getHSISpeedInHz(void) const;
    unsigned getHSESpeedInHz(void) const;

    PllSource_t getPllSource(void) const;
    unsigned getPllInputSpeedInHz(void) const;
    unsigned getPllVcoSpeedInHz(void) const;
    unsigned getPllSpeedInHz(void) const;

    AHBPrescaler_t getAHBPrescaler(void) const;
    unsigned getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) const;

    APBPrescaler_t getAPB1Prescaler(void) const;
    APBPrescaler_t getAPB2Prescaler(void) const;
    
    unsigned getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const;
    
    void setupFlash(void) const;
    void setupPower(void) const;
}; /* class RccViaSTM32F4 */

} /* namespace RccViaSTM32F4 */

#endif /* _RCC_STM32F4_HPP_89fcdf0c_4b73_427d_92e2_93986e806bb5 */
