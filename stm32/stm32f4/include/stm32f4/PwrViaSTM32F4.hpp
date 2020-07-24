/*-
 * $Copyright$
-*/

#ifndef _PWR_STM32F4_HPP_87c40e74_a65b_400f_a13d_78b01ad6511d
#define _PWR_STM32F4_HPP_87c40e74_a65b_400f_a13d_78b01ad6511d

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <assert.h>

/******************************************************************************/
namespace stm32 {
    namespace f4 {
/******************************************************************************/

class Pwr {
public:
    constexpr Pwr(PWR_TypeDef * const p_pwr)
      : m_pwr(p_pwr) {
    }

    void setupVoltageScaling(void) const {
    /* FIXME CPU-type specific pre-processor macro */
    #if defined(PWR_CR_VOS)
        m_pwr->CR |= PWR_CR_VOS;
    #elif defined(PWR_CR1_VOS)
        m_pwr->CR1 |= PWR_CR1_VOS;
    #else
        #error Unhandled else case here!
    #endif
    }

private:
    PWR_TypeDef * const m_pwr;
}; /* class Pwr */

/******************************************************************************/
    } /* namespace f4 */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _PWR_STM32F4_HPP_87c40e74_a65b_400f_a13d_78b01ad6511d */
