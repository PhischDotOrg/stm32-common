/*-
 * $Copyright$
-*/

#ifndef _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78
#define _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78

#include <stm32f4xx.h>

/******************************************************************************/
namespace stm32 {
    namespace f4 {
/******************************************************************************/

class Flash {
    FLASH_TypeDef &m_flash;

public:
    Flash(FLASH_TypeDef * const p_flash)
      : m_flash(*p_flash) {
        setupSafeMode();
    }

    typedef enum {
        e_FlashLatency_0WaitStates = 0,
        e_FlashLatency_1WaitStates = 1,
        e_FlashLatency_2WaitStates = 2,
        e_FlashLatency_3WaitStates = 3,
        e_FlashLatency_4WaitStates = 4,
        e_FlashLatency_5WaitStates = 5,
        e_FlashLatency_6WaitStates = 6,
        e_FlashLatency_7WaitStates = 7, 
    } WaitStates_t;
    
    void enableICache(const bool p_enable) const {
        m_flash.ACR &= ~FLASH_ACR_ICEN;
        if (p_enable) {
            flushICache();
            
            m_flash.ACR |= FLASH_ACR_ICEN;
        }
    }

    void enableDCache(const bool p_enable) const {
        m_flash.ACR &= ~FLASH_ACR_DCEN;
        if (p_enable) {
            flushDCache();
            
            m_flash.ACR |= FLASH_ACR_DCEN;
        }    
    }

    void enablePrefetch(const bool p_enable) const {
        if (p_enable) {
            m_flash.ACR |= FLASH_ACR_PRFTEN;
        } else {
            m_flash.ACR &= ~FLASH_ACR_PRFTEN;        
        }
    }

    void setupLatency(const WaitStates_t p_latency) const {
        m_flash.ACR |= FLASH_ACR_LATENCY;
        m_flash.ACR ^= (~p_latency) & FLASH_ACR_LATENCY;
    }

    void setupLatency(unsigned p_sysclkSpeedInHz) const {
        /* FIXME Assuming 3V Operation for now */
        if (p_sysclkSpeedInHz <= 30 * 1000 * 1000) {
            setupLatency(WaitStates_t::e_FlashLatency_0WaitStates);
        } else if (p_sysclkSpeedInHz <= 64 * 1000 * 1000) {
            setupLatency(WaitStates_t::e_FlashLatency_1WaitStates);
        } else if (p_sysclkSpeedInHz <= 90 * 1000 * 1000) {
            setupLatency(WaitStates_t::e_FlashLatency_2WaitStates);
        } else if (p_sysclkSpeedInHz <= 120 * 1000 * 1000) {
            setupLatency(WaitStates_t::e_FlashLatency_3WaitStates);        
        } else if (p_sysclkSpeedInHz <= 150 * 1000 * 1000) {
            setupLatency(WaitStates_t::e_FlashLatency_4WaitStates);        
        } else /* if (p_sysclkSpeedInHz <= 168 * 1000 * 1000) */ {
            setupLatency(WaitStates_t::e_FlashLatency_5WaitStates);        
        }
    }

private:
    void setupSafeMode(void) const {
        enableICache(false);
        flushICache();
        
        enableDCache(false);
        flushDCache();

        enablePrefetch(false);
        
        setupLatency(e_FlashLatency_7WaitStates);
    }

    void flushICache(void) const {
        m_flash.ACR |= FLASH_ACR_ICRST;
        m_flash.ACR &= ~FLASH_ACR_ICRST;
    }

    void flushDCache(void) const {
        m_flash.ACR |= FLASH_ACR_DCRST;
        m_flash.ACR &= ~FLASH_ACR_DCRST;
    }
}; /* class Flash */

/******************************************************************************/
    } /* namespace f4 */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78 */
