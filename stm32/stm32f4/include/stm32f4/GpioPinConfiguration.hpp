/*-
 * $Copyright$
 */
#ifndef _STM32F4_GPIO_PIN_CONFIGURATION_HPP_96CBD113_094F_4015_9A8B_5575761C84CB
#define _STM32F4_GPIO_PIN_CONFIGURATION_HPP_96CBD113_094F_4015_9A8B_5575761C84CB

#include <stm32f4xx.h>
#include <stm32/Gpio.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f4 {
/*****************************************************************************/

template<
    typename EngineT
>
struct IoMuxT {
    static constexpr std::nullptr_t m_alternateFn = nullptr;
};

/*****************************************************************************/
struct GpioPinConfiguration {
    enum class Mode_e {
        e_Input     = 0,
        e_Output    = 1,
        e_Alternate = 2,
        e_Analog    = 3,
    };

    enum class Termination_e {
        e_None = 0,
        e_PullUp = 1,
        e_PullDown = 2
    };

    enum class Function_e : uint32_t {
        e_Gpio = 0,
        e_Mco = 0,
        e_Tim1 = 1,
        e_Tim2 = 1,
        e_Tim3 = 2,
        e_Tim4 = 2,
        e_Tim5 = 2,
        e_Tim8 = 3,
        e_Tim9 = 3,
        e_Tim10 = 3,
        e_Tim11 = 3,
//        e_I2c1 = 1,
//        e_I2c2 = 1,
//        e_I2c3 = 1,
        e_SPI1 = 5,
        e_SPI2 = 5,
        e_SPI3 = 6,
        e_USART1 = 7,
        e_USART2 = 7,
        e_Uart3 = 7,
        e_Uart4 = 8,
        e_Uart5 = 8,
        e_USART6 = 8,
        e_UsbFs = 10,
        e_UsbHs = 10,
        e_Alternate,
    };

    static void
    enable(GPIO_TypeDef &p_engine, uint8_t p_pin, Mode_e p_mode, Termination_e p_termination) {
        unsigned offset = (p_pin * 2);
        p_engine.MODER &= ~(0x3 << offset);
        p_engine.MODER |= static_cast<unsigned>(p_mode) << offset;

        p_engine.PUPDR &= ~(0x3 << offset);
        p_engine.PUPDR |= static_cast<unsigned>(p_termination) << offset;

        p_engine.OSPEEDR &= ~(0x3 << offset);
        p_engine.OSPEEDR |= (0x2 << offset);
    }

    template<typename EngineT>
    static constexpr void
    selectAlternateFn(GPIO_TypeDef &p_gpioEngine, uint8_t p_pin, const EngineT & /* p_engine */) {
        configureAlternateFn(p_gpioEngine, p_pin, IoMuxT<EngineT>::m_alternateFn);
    }

private:
    static void
    configureAlternateFn(GPIO_TypeDef &p_engine, uint8_t p_pin, Function_e p_function) {
        unsigned fnoffs = ((p_pin & 0x07) * 4);
        p_engine.AFR[p_pin >> 3] &= ~(0xF << fnoffs);
        p_engine.AFR[p_pin >> 3] |= static_cast<unsigned>(p_function) << fnoffs;
    }
}; /* class GpioPinConfiguration */

#define MAP_IO_ENGINE(Engine)                                                           \
template<> struct IoMuxT< EngineT< (Engine ## _BASE) > > {                              \
    static constexpr GpioPinConfiguration::Function_e m_alternateFn = GpioPinConfiguration::Function_e::e_##Engine; \
}

#if defined(SPI1_BASE)
    MAP_IO_ENGINE(SPI1);
#endif
#if defined(SPI2_BASE)
    MAP_IO_ENGINE(SPI2);
#endif
#if defined(SPI3_BASE)
    MAP_IO_ENGINE(SPI3);
#endif

#if defined(USART1_BASE)
MAP_IO_ENGINE(USART1);
#endif
#if defined(USART2_BASE)
MAP_IO_ENGINE(USART2);
#endif
#if defined(USART6_BASE)
MAP_IO_ENGINE(USART6);
#endif


/*****************************************************************************/
    } /* namespace f4 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32F1_GPIO_PIN_CONFIGURATION_HPP_592224C2_A117_476E_853E_40800D857230 */
