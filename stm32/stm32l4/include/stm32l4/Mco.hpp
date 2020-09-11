/*-
 * $Copyright$
 */
#ifndef _STM32L4_MCO_HPP_8A391BFA_0EB9_4587_8EC8_0197651F71B5
#define _STM32L4_MCO_HPP_8A391BFA_0EB9_4587_8EC8_0197651F71B5

#include <stdint.h>

/*****************************************************************************/
namespace stm32 {
    namespace l4 {
/*****************************************************************************/

struct Mco {
    typedef enum MCOPrescaler_e : uint8_t {
        e_MCOPre_None   = 0b000,
        e_MCOPre_2      = 0b100,
        e_MCOPre_3      = 0b101,
        e_MCOPre_4      = 0b110,
        e_MCOPre_5      = 0b111,
    } MCOPrescaler_t;

    typedef enum class MCO1Output_e : uint8_t {
        e_HSI    = 0b00,
        e_LSE    = 0b01,
        e_HSE    = 0b10,
        e_PLL    = 0b11,
        e_None,
    } MCO1Output_t;

    using MCOOutput_t = MCO1Output_t;

    typedef enum class MCO2Output_e {
        e_Sysclk = 0b00,
        e_PllI2S = 0b01,
        e_HSE    = 0b10,
        e_PLL    = 0b11,
        e_None
    } MCO2Output_t;

protected:
    template<
        typename RCC_TypeDef,
        typename MCOPinT
    >
    static void
    setMCO(RCC_TypeDef &p_rcc, const MCOPinT &p_mcoPin, const MCO1Output_t p_output, const MCOPrescaler_t p_prescaler) {
        p_rcc.CFGR &= ~RCC_CFGR_MCOPRE_Msk;
        p_rcc.CFGR |= (static_cast<unsigned>(p_prescaler) << RCC_CFGR_MCOPRE_Pos) & RCC_CFGR_MCOPRE_Msk;

        p_rcc.CFGR &= ~RCC_CFGR_MCOSEL_Msk;
        p_rcc.CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCOSEL_Pos) & RCC_CFGR_MCOSEL_Msk;

        if (p_output != MCOOutput_t::e_None) {
            p_mcoPin.enable();
        } else {
            p_mcoPin.enable();
        }
    }
}; /* struct Mco */

/*****************************************************************************/
    } /* namespace l4 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32L4_MCO_HPP_8A391BFA_0EB9_4587_8EC8_0197651F71B5 */
