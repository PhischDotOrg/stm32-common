/*-
 * $Copyright$
 */
#ifndef _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A
#define _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A

#include <stm32f4xx.h>

#include <stm32f1/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32f1/GpioPinConfiguration.hpp>
#include <stm32f1/Nvic.hpp>
#include <stm32f1/PllCfg.hpp>
#include <stm32f1/Pwr.hpp>
#include <stm32f1/Rcc.hpp>
#include <stm32f1/Scb.hpp>

namespace stm32 {
    namespace f1 {
        namespace f103 {
            struct Cpu {
                using Flash     = f1::Flash;
                using Nvic      = f1::NvicT<f1::Scb>;
                using PllCfg    = f1::PllCfg;
                using Pwr       = f1::Pwr;
                using Rcc       = f1::Rcc;
                using Scb       = f1::Scb;

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f1::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, stm32::f1::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, stm32::f1::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, stm32::f1::GpioPinConfiguration>;
                    using D     = ::stm32::GpioT<Rcc, GPIOD_BASE, stm32::f1::GpioPinConfiguration>;
                    using E     = ::stm32::GpioT<Rcc, GPIOE_BASE, stm32::f1::GpioPinConfiguration>;
                }; /* struct gpio */

                struct Uart {
                    template<typename PinT> using Usart1    = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                    template<typename PinT> using Usart2    = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                    template<typename PinT> using Usart3    = ::stm32::UartT<Rcc, USART3_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                }; /* struct Uart */
            }; /* struct Cpu */
        } /* namespace f103 */
    } /* namespace f1 */

    using Cpu = f1::f103::Cpu;

    using Gpio = Cpu::Gpio;
    using GpioEngine = Cpu::Gpio::Engine;
    using Rcc = Cpu::Rcc;
    using Uart = Cpu::Uart;

    MAP_RCC_ENGINE(GPIOA);
    MAP_RCC_ENGINE(GPIOB);
    MAP_RCC_ENGINE(GPIOC);
    MAP_RCC_ENGINE(GPIOD);

    MAP_RCC_ENGINE(USART1);
    MAP_RCC_ENGINE(USART2);
    MAP_RCC_ENGINE(USART3);
} /* namespace stm32 */

#endif /* _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A */
