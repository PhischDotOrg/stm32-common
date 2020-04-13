/*-
 * $Copyright$
-*/

#ifndef _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78
#define _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <assert.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class FlashViaSTM32F4 {
public:
    FlashViaSTM32F4(FLASH_TypeDef * const p_flash);
    ~FlashViaSTM32F4();
    
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
    
    void enableICache(const bool p_enable) const;
    void enableDCache(const bool p_enable) const;
    void enablePrefetch(const bool p_enable) const;

    void setupLatency(const WaitStates_t p_latency) const;

private:
    FLASH_TypeDef * const m_flash;

    void setupSafeMode(void) const;
    void flushICache(void) const;
    void flushDCache(void) const;
}; /* class FlashViaSTM32F4 */

} /* namespace devices */

#endif /* _FLASH_STM32F4_HPP_8883230b_7622_4473_b86e_dbef18b89c78 */
