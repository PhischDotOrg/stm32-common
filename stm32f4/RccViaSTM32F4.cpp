/*-
 * $Copyright$
-*/

#ifndef _RCC_STM32F4_CPP_596f9f7e_15f5_42cb_b1a1_c92ef521b265
#define _RCC_STM32F4_CPP_596f9f7e_15f5_42cb_b1a1_c92ef521b265

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

extern uint32_t SystemCoreClock;

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/RccViaSTM32F4.hpp>
#include <assert.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::RccViaSTM32F4(RCC_TypeDef * const p_rcc, PllConfiguration &p_pllCfg, FlashViaSTM32F4 &p_flash, PwrViaSTM32F4 &p_pwr)
  : m_rcc(p_rcc), m_pllCfg(p_pllCfg), m_flash(p_flash), m_pwr(p_pwr) {
    setupSafeMode();

    enableHSE((m_pllCfg.m_pllSource == e_PllSourceHSE) || (m_pllCfg.m_sysclkSource == e_SysclkHSE));
    enablePLL(m_pllCfg.m_sysclkSource == e_SysclkPLL);

    /*
     * Set up dividers for AHB, APB1 (low-speed peripheral) and APB2 (high
     * speed peripheral) busses.
     */
    m_rcc->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    if (m_pllCfg.m_ahbPrescaler != e_AHBPrescaler_None) {
        m_rcc->CFGR |= (m_pllCfg.m_ahbPrescaler  <<  4) & RCC_CFGR_HPRE;
        m_rcc->CFGR |= RCC_CFGR_HPRE_3;
    }
    
    if (m_pllCfg.m_apb1Prescaler != e_APBPrescaler_None) {
        m_rcc->CFGR |= (m_pllCfg.m_apb1Prescaler << 10) & RCC_CFGR_PPRE1;
        m_rcc->CFGR |= RCC_CFGR_PPRE1_2;
    }

    if (m_pllCfg.m_apb2Prescaler != e_APBPrescaler_None) {
        m_rcc->CFGR |= (m_pllCfg.m_apb2Prescaler << 13) & RCC_CFGR_PPRE2;
        m_rcc->CFGR |= RCC_CFGR_PPRE2_2;
    }

    setupFlash();

    setupPower();

    /* Finally, switch the clock source to whatever was requested */
    m_rcc->CFGR |= (m_pllCfg.m_sysclkSource << 0) & RCC_CFGR_SW;
    while ((m_rcc->CFGR & RCC_CFGR_SWS) != ((m_pllCfg.m_sysclkSource << 2) & RCC_CFGR_SWS));

    /* Turn off HSI, if not required */
    enableHSI((m_pllCfg.m_pllSource == e_PllSourceHSI) || (m_pllCfg.m_sysclkSource == e_SysclkHSI));
    
    SystemCoreClockUpdate();
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::~RccViaSTM32F4() {
}


/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::setupSafeMode() const {
    /* Enable HSI and wait until it's ready */
    m_rcc->CR |= RCC_CR_HSION;
    while (!(m_rcc->CR & RCC_CR_HSIRDY)) ;

    /* Set up the internal oscillator as the system clock */
    m_rcc->CFGR &= ~RCC_CFGR_SW;
    while (m_rcc->CFGR & RCC_CFGR_SWS);

    /* Disable external Oscillator, the clock security system and the internal PLL */
    m_rcc->CR &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enableHSE(const bool p_enable) const {
    m_rcc->CR &= ~RCC_CR_HSEON;

    /*
     * If enable is requested, then enable external oscillator input and wait
     * for signal to settle.
     */
    if (p_enable) {
        m_rcc->CR |= RCC_CR_HSEON;
        while (!(m_rcc->CR & RCC_CR_HSERDY));
    }
}


/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enableHSI(const bool p_enable) const {
    m_rcc->CR &= ~RCC_CR_HSION;

    /*
     * If enable is requested, then enable the internal oscillator and wait for
     * it to become ready.
     */
    if (p_enable) {
        m_rcc->CR |= RCC_CR_HSION;
        while (!(m_rcc->CR & RCC_CR_HSIRDY));
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enablePLL(const bool p_enable) const {
    m_rcc->CR &= ~RCC_CR_PLLON;
    m_rcc->PLLCFGR = 0x24003010; /* Set to Power-on Reset value */
    
    if (p_enable) {
        m_rcc->PLLCFGR = 
                ((m_pllCfg.m_pllQ << 24) & RCC_PLLCFGR_PLLQ)
              | ((m_pllCfg.m_pllP << 16) & RCC_PLLCFGR_PLLP)
              | ((m_pllCfg.m_pllN <<  6) & RCC_PLLCFGR_PLLN)
              | ((m_pllCfg.m_pllM <<  0) & RCC_PLLCFGR_PLLM);
        m_rcc->PLLCFGR |= (m_pllCfg.m_pllSource == e_PllSourceHSE) ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI;        

        m_rcc->CR |= RCC_CR_PLLON;
        while (!(m_rcc->CR & RCC_CR_PLLRDY));    
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::setupFlash(void) const {
    m_flash.enableICache(true);
    m_flash.enableDCache(true);
    m_flash.enablePrefetch(true);

    unsigned targetSpeed;
    switch (m_pllCfg.m_sysclkSource) {
    case e_SysclkHSI:
        targetSpeed = getHSISpeedInHz();
        break;
    case e_SysclkHSE:
        targetSpeed = getHSESpeedInHz();
        break;
    case e_SysclkPLL:
        targetSpeed = getPllSpeedInHz();
        break;
    default:
        while (1);
        break;
    }
    
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

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::setupPower(void) const {
    unsigned targetSpeed;

    switch (m_pllCfg.m_sysclkSource) {
    case e_SysclkHSI:
        targetSpeed = getHSISpeedInHz();
        break;
    case e_SysclkHSE:
        targetSpeed = getHSESpeedInHz();
        break;
    case e_SysclkPLL:
        targetSpeed = getPllSpeedInHz();
        break;
    default:
        while (1);
        break;
    }

    m_pwr.setupVoltageScaling(targetSpeed);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enable(const FunctionAPB1_t p_function) const {
    this->m_rcc->APB1ENR |= p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void 
RccViaSTM32F4::disable(const FunctionAPB1_t p_function) const {
    this->m_rcc->APB1ENR &= ~p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enable(const FunctionAPB2_t p_function) const {
    this->m_rcc->APB2ENR |= p_function;    
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::disable(const FunctionAPB2_t p_function) const {
    this->m_rcc->APB2ENR &= ~p_function;    
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enable(const FunctionAHB1_t p_function) const {
    this->m_rcc->AHB1ENR |= p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::RccViaSTM32F4::disable(const FunctionAHB1_t p_function) const {
    this->m_rcc->AHB1ENR &= ~p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::enable(const FunctionAHB2_t p_function) const {
    this->m_rcc->AHB2ENR |= p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
RccViaSTM32F4::RccViaSTM32F4::disable(const FunctionAHB2_t p_function) const {
    this->m_rcc->AHB2ENR &= ~p_function;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getClockSpeed(const FunctionAPB1_t /* p_function */) const {
    return this->getApb1SpeedInHz();
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getClockSpeed(const FunctionAPB2_t /* p_function */) const {
    return this->getApb2SpeedInHz();
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getAhbSpeedInHz(void) const {
    unsigned prescaler = this->getAHBPrescalerValue(this->getAHBPrescaler());

    return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getApb1SpeedInHz(void) const {
    unsigned prescaler = this->getAPBPrescalerValue(this->getAPB1Prescaler());

    return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
    
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getApb2SpeedInHz(void) const {
    unsigned prescaler = this->getAPBPrescalerValue(this->getAPB2Prescaler());

    return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;    
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::SysclkSource_t
RccViaSTM32F4::getSysclkSource(void) const {
    SysclkSource_t src = static_cast<SysclkSource_t>((m_rcc->CFGR & RCC_CFGR_SWS) >> 2);

    return src;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getHSISpeedInHz(void) const {
    return m_pllCfg.m_hsiSpeedInHz;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getHSESpeedInHz(void) const {
    return m_pllCfg.m_hseSpeedInHz;    
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getSysclkSpeedInHz(void) const {
    SysclkSource_t sysclkSrc = getSysclkSource();
    unsigned speed;

    switch (sysclkSrc) {
    case e_SysclkHSI:
        speed = getHSISpeedInHz();
        break;
    case e_SysclkHSE:
        speed = getHSESpeedInHz();
        break;
    case e_SysclkPLL:
        speed = getPllSpeedInHz();
        break;
    default:
        speed = 0;
    }

    return speed;
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::PllSource_t
RccViaSTM32F4::getPllSource(void) const {
    return static_cast<PllSource_t>((m_rcc->PLLCFGR & RCC_PLLCFGR_PLLSRC) >> 22);
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getPllInputSpeedInHz(void) const {
    PllSource_t source = getPllSource();
    unsigned speed;

    switch (source) {
    case e_PllSourceHSI:
        speed = getHSISpeedInHz();
        break;
    case e_PllSourceHSE:
        speed = getHSESpeedInHz();
        break;
    default:
        speed = -1;
    }

    return speed;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getPllVcoSpeedInHz(void) const {
    unsigned input = getPllInputSpeedInHz();

    unsigned plln = ((m_rcc->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6) & 0x01FF;
    assert((plln > 1) && (plln < 433));

    unsigned pllm = ((m_rcc->PLLCFGR & RCC_PLLCFGR_PLLM) >> 0) & 0x003F;
    assert(pllm > 1);

    unsigned vco = ((input * plln) + (pllm / 2)) / pllm;

    assert((vco >= 100000000) && (vco <= 432000000));

    return vco;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getPllSpeedInHz(void) const {
    unsigned vco = getPllVcoSpeedInHz();

    unsigned pllp = 2 << (((m_rcc->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) & 0x3);

    return (vco + (pllp / 2)) / pllp;
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::AHBPrescaler_t
RccViaSTM32F4::getAHBPrescaler(void) const {
    AHBPrescaler_t prescaler;

    if (m_rcc->CFGR & RCC_CFGR_HPRE_3) {
        prescaler = e_AHBPrescaler_None;
    } else {
        unsigned hpre = ((m_rcc->CFGR & RCC_CFGR_HPRE) >> 4) & 0x7;
        prescaler = static_cast<AHBPrescaler_t>(hpre);
    }

    return prescaler;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) const {
    unsigned value;

    switch (p_prescaler) {
    case e_AHBPrescaler_Div2:
    case e_AHBPrescaler_Div4:
    case e_AHBPrescaler_Div8:
    case e_AHBPrescaler_Div16:
        value = 2 << p_prescaler;
        break;
    case e_AHBPrescaler_Div64:
    case e_AHBPrescaler_Div128:
    case e_AHBPrescaler_Div256:
    case e_AHBPrescaler_Div512:
        value = 4 << p_prescaler;
        break;
    case e_AHBPrescaler_None:
        value = 1;
        break;
    default:
        while (1);
    }

    return value;
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::APBPrescaler_t
RccViaSTM32F4::getAPB1Prescaler(void) const {
    APBPrescaler_t prescaler;        

    if (!(m_rcc->CFGR & RCC_CFGR_PPRE1_2)) {
        prescaler = e_APBPrescaler_None;
    } else {
        unsigned ppre = ((m_rcc->CFGR & RCC_CFGR_PPRE1) >> 10) & 0x3;
        prescaler = static_cast<APBPrescaler_t>(ppre);
    }

    return prescaler;
}

/*******************************************************************************
 * 
 ******************************************************************************/
RccViaSTM32F4::APBPrescaler_t
RccViaSTM32F4::getAPB2Prescaler(void) const {
    APBPrescaler_t prescaler;        

    if (!(m_rcc->CFGR & RCC_CFGR_PPRE2_2)) {
        prescaler = e_APBPrescaler_None;
    } else {
        unsigned ppre = ((m_rcc->CFGR & RCC_CFGR_PPRE2) >> 13) & 0x3;
        prescaler = static_cast<APBPrescaler_t>(ppre);
    }

    return prescaler;
}

/*******************************************************************************
 * 
 ******************************************************************************/
unsigned
RccViaSTM32F4::getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const {
    unsigned value;

    switch (p_prescaler) {
    case e_APBPrescaler_Div2:
    case e_APBPrescaler_Div4:
    case e_APBPrescaler_Div8:
    case e_APBPrescaler_Div16:
        value = 2 << p_prescaler;
        break;
    case e_APBPrescaler_None:
        value = 1;
        break;
    default:
        while (1);
    }

    return value;
}

} /* namespace devices */


#endif /* _RCC_STM32F4_CPP_596f9f7e_15f5_42cb_b1a1_c92ef521b265 */
