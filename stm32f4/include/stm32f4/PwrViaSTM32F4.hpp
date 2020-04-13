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

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class PwrViaSTM32F4 {
public:
    PwrViaSTM32F4(PWR_TypeDef * const p_pwr);
    ~PwrViaSTM32F4();

    void setupVoltageScaling(unsigned p_systemSpeed) const;

private:
    PWR_TypeDef * const m_pwr;
}; /* class PwrViaSTM32F4 */

} /* namespace devices */

#endif /* _PWR_STM32F4_HPP_87c40e74_a65b_400f_a13d_78b01ad6511d */
