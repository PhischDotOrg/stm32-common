/*-
 * $Copyright$
 */

#ifndef _STM32F4_MCO_HPP_ED031174_1CD8_49D3_837E_7B824965562D
#define _STM32F4_MCO_HPP_ED031174_1CD8_49D3_837E_7B824965562D

#include <cstdint>
#include <stm32/Gpio.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f4 {
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
        p_rcc.CFGR &= ~RCC_CFGR_MCO1PRE_Msk;
        p_rcc.CFGR |= (p_prescaler << RCC_CFGR_MCO1PRE_Pos) & RCC_CFGR_MCO1PRE_Msk;

        p_rcc.CFGR &= ~RCC_CFGR_MCO1_Msk;
        p_rcc.CFGR |= (static_cast<unsigned>(p_output) << RCC_CFGR_MCO1_Pos) & RCC_CFGR_MCO1_Msk;

        if (p_output != MCO1Output_t::e_None) {
            /* FIXME Setting MCO Output probably doesn't work anymore on STM32F4 */
            // p_mcoPin.enable(stm32::GpioEngine::e_Alternate, stm32::GpioEngine::e_None, stm32::GpioEngine::e_Mco);
            p_mcoPin.enable();
        } else {
            p_mcoPin.disable();
        }
    }
}; /* struct Mco */

/*****************************************************************************/
    } /* namespace f4 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32F4_MCO_HPP_ED031174_1CD8_49D3_837E_7B824965562D */
