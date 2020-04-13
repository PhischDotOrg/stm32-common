/*-
 * $Copyright$
-*/

#ifndef _FLASH_STM32F4_HPP_e2433317_90b1_4673_a028_2cf8b222709a
#define _FLASH_STM32F4_HPP_e2433317_90b1_4673_a028_2cf8b222709a

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/FlashViaSTM32F4.hpp>
#include <assert.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
FlashViaSTM32F4::FlashViaSTM32F4(FLASH_TypeDef * const p_flash) : m_flash(p_flash) {
    setupSafeMode();
}

/*******************************************************************************
 * 
 ******************************************************************************/
FlashViaSTM32F4::~FlashViaSTM32F4() {
    setupSafeMode();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::setupSafeMode(void) const {
    enableICache(false);
    flushICache();
    
    enableDCache(false);
    flushDCache();

    enablePrefetch(false);
    
    setupLatency(e_FlashLatency_7WaitStates);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::enableICache(const bool p_enable) const {
    m_flash->ACR &= ~FLASH_ACR_ICEN;
    if (p_enable) {
        flushICache();
        
        m_flash->ACR |= FLASH_ACR_ICEN;
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::flushICache(void) const {
    m_flash->ACR |= FLASH_ACR_ICRST;
    m_flash->ACR &= ~FLASH_ACR_ICRST;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::enableDCache(const bool p_enable) const {
    m_flash->ACR &= ~FLASH_ACR_DCEN;
    if (p_enable) {
        flushDCache();
        
        m_flash->ACR |= FLASH_ACR_DCEN;
    }    
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::flushDCache(void) const {
    m_flash->ACR |= FLASH_ACR_DCRST;
    m_flash->ACR &= ~FLASH_ACR_DCRST;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::enablePrefetch(const bool p_enable) const {
    if (p_enable) {
        m_flash->ACR |= FLASH_ACR_PRFTEN;
    } else {
        m_flash->ACR &= ~FLASH_ACR_PRFTEN;        
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
FlashViaSTM32F4::setupLatency(const WaitStates_t p_latency) const {
    m_flash->ACR |= FLASH_ACR_LATENCY;
    m_flash->ACR ^= (~p_latency) & FLASH_ACR_LATENCY;
}

} /* namespace devices */

#endif /* _FLASH_STM32F4_HPP_e2433317_90b1_4673_a028_2cf8b222709a */