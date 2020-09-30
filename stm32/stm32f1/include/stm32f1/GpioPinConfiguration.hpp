/*-
 * $Copyright$
 */
#ifndef _STM32F1_GPIO_PIN_CONFIGURATION_HPP_592224C2_A117_476E_853E_40800D857230
#define _STM32F1_GPIO_PIN_CONFIGURATION_HPP_592224C2_A117_476E_853E_40800D857230

#include <stm32f4xx.h>
#include <stm32/Gpio.hpp>

#include <cassert>

#include <array>
#include <utility>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
/*****************************************************************************/

class GpioPinConfiguration {
public:
    enum class Function_e {

    };

    enum class Mode_e : uint8_t {
        e_Input,
        e_Output,
        e_Alternate,
        e_Analog,
    };

    enum class Termination_e : uint8_t {
        e_None,
        e_PullUp,
        e_PullDown,
    };

private:
    enum class PortModeBits_e : uint8_t {
        e_Input         = 0b00,
        e_Output10MHz   = 0b01,
        e_Output2MHz    = 0b10,
        e_Output50MHz   = 0b11,
    };

    enum class PortConfBits_e : uint8_t {
        /* Input */
        e_InputAnalog       = 0b00,
        e_InputFloating     = 0b01,
        e_InputPullUpDown   = 0b10,
        /* Output */
        e_OutputPushPull    = 0b00,
        e_OutputOpenDrain   = 0b01,
        e_AlternatePushPull = 0b10,
        e_AlternateOpenDrain= 0b11,
    };

    struct ModeMapping_s {
        const Mode_e            m_mode;
        const Termination_e     m_termination;
        const PortModeBits_e    m_portModeBits;
        const PortConfBits_e    m_portConfBits;
    };
    
    static constexpr std::array<struct ModeMapping_s, 8> m_modeMapping { {
        { Mode_e::e_Input,      Termination_e::e_None,      PortModeBits_e::e_Input,        PortConfBits_e::e_InputFloating }, /* Keep this first! */
        { Mode_e::e_Input,      Termination_e::e_PullUp,    PortModeBits_e::e_Input,        PortConfBits_e::e_InputPullUpDown },
        { Mode_e::e_Input,      Termination_e::e_PullDown,  PortModeBits_e::e_Input,        PortConfBits_e::e_InputPullUpDown },
        { Mode_e::e_Output,     Termination_e::e_None,      PortModeBits_e::e_Output10MHz,  PortConfBits_e::e_OutputPushPull },
        { Mode_e::e_Output,     Termination_e::e_PullUp,    PortModeBits_e::e_Output10MHz,  PortConfBits_e::e_OutputOpenDrain },
        { Mode_e::e_Alternate,  Termination_e::e_None,      PortModeBits_e::e_Output50MHz,  PortConfBits_e::e_AlternatePushPull },
        { Mode_e::e_Alternate,  Termination_e::e_PullUp,    PortModeBits_e::e_Output50MHz,  PortConfBits_e::e_AlternateOpenDrain },
        { Mode_e::e_Analog,     Termination_e::e_None,      PortModeBits_e::e_Input,        PortConfBits_e::e_InputAnalog },
    } };

    static constexpr
    decltype(m_modeMapping)::const_iterator
    mapGpioMode(Mode_e p_mode, Termination_e p_termination) {
        auto iter = m_modeMapping.begin();
        for ( ; iter != m_modeMapping.end(); iter++) {
            if (((*iter).m_mode == p_mode) && ((*iter).m_termination == p_termination)) {
                break;
            }
        }

        /*
         * If the combination of p_mode and p_termination does not map to a valid entry in
         * the m_modeMapping table, then bail out with an assertion.
         *
         * In non-debug builds, the assertion won't do anything. In that case, return the
         * first entry in the m_modeMapping table.
         * 
         * This should configure the pin as a floating input which is the power-on reset
         * configuration for an IO Pin on the STM32F103.
         */
        assert(iter != m_modeMapping.end());
        if (iter == m_modeMapping.end()) {
            iter = m_modeMapping.begin();
        }

        return iter;
    }

public:
    static void
    enable(GPIO_TypeDef &p_engine, uint8_t p_pin, Mode_e p_mode, Termination_e p_termination) {
        volatile uint32_t * const cr = p_pin < 8 ? &p_engine.CRL : &p_engine.CRH;

        const unsigned modeOffs = 4 * (p_pin % 8) + 0;
        const unsigned confOffs = 4 * (p_pin % 8) + 2;

        decltype(m_modeMapping)::const_iterator iter = mapGpioMode(p_mode, p_termination);

        const PortModeBits_e modeBits = (*iter).m_portModeBits;
        const PortConfBits_e confBits = (*iter).m_portConfBits;

        *cr &= ~((0b11 << modeOffs) | (0b11 << confOffs));
        *cr |= (static_cast<unsigned>(modeBits) << modeOffs) | (static_cast<unsigned>(confBits) << confOffs);

        if (p_mode == Mode_e::e_Input) {
            if (p_termination == Termination_e::e_PullUp) {
                p_engine.ODR |= (1 << p_pin);
            } else if (p_termination == Termination_e::e_PullDown) {
                p_engine.ODR &= ~(1 << p_pin);
            }
        }
    }

    template<typename EngineT>
    static void
    selectAlternateFn(GPIO_TypeDef &p_engine, uint8_t p_pin, const EngineT & /* p_engineFn */) {
        enable(p_engine, p_pin, Mode_e::e_Alternate, Termination_e::e_None);
    }
}; /* class GpioPinConfiguration */

/*****************************************************************************/
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32F1_GPIO_PIN_CONFIGURATION_HPP_592224C2_A117_476E_853E_40800D857230 */
