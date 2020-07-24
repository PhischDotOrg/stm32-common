/*-
 * $Copyright$
 */

#ifndef _STM32F1_MCO_HPP_D25644D9_FB11_40DC_897E_944F72737A82
#define _STM32F1_MCO_HPP_D25644D9_FB11_40DC_897E_944F72737A82

#include <cstdint>
#include <stm32/Gpio.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
/*****************************************************************************/

struct McoPolicy {
    typedef enum MCOOutput_e : uint8_t {
        e_None      = 0b000,
        e_Sysclk    = 0b100,
        e_HSI       = 0b101,
        e_HSE       = 0b110,
        e_PLL       = 0b111,
    } MCOOutput_t;

    using MCO1Output_t = MCOOutput_t;

    /*
        * There is no MCO Prescaler on the STM32F103xB CPU; this type is only there to
        * keep the interface similar to the STM32F4-line CPU Code.
        */
    typedef enum MCOPrescaler_e {
        e_MCOPre_None,
    } MCOPrescaler_t;

protected:
    template<typename MCOPinT>
    static void
    setMCO(RCC_TypeDef &p_rcc, const MCOPinT &p_mcoPin, const MCOOutput_t p_output, const MCOPrescaler_t /* p_prescaler */ = MCOPrescaler_t::e_MCOPre_None) {
        p_rcc.CFGR &= ~RCC_CFGR_MCO_Msk;
        p_rcc.CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCO_Pos) & RCC_CFGR_MCO_Msk;

        if (p_output != MCOOutput_t::e_None) {
            p_mcoPin.enable();
        } else {
            p_mcoPin.disable();
        }
    }
}; /* struct McoPolicy */

/*****************************************************************************/
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32F1_MCO_HPP_D25644D9_FB11_40DC_897E_944F72737A82 */
