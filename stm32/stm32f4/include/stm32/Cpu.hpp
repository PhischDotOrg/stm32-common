/*-
 * $Copyright$
 */
#ifndef _STM32_CPU_HPP_F936BB8E_9326_416B_BD61_9F6F683DFF3D
#define _STM32_CPU_HPP_F936BB8E_9326_416B_BD61_9F6F683DFF3D

#include <stm32f4xx.h>

#include <stm32/RccEngine.hpp>

#include <stm32f4/FlashViaSTM32F4.hpp>
#include <stm32/Gpio.hpp>
#include <stm32f4/GpioPinConfiguration.hpp>
#include <stm32f4/NvicViaSTM32F4.hpp>
#include <stm32f4/PllCfg.hpp>
#include <stm32f4/PwrViaSTM32F4.hpp>
#include <stm32f4/Rcc.hpp>
#include <stm32f4/ScbViaSTM32F4.hpp>
#include <stm32/Uart.hpp>

namespace stm32 {
    namespace f4 {
        namespace f407 {
            struct Cpu {
                using Flash     = ::stm32::f4::Flash;
                using Nvic      = ::devices::NvicViaSTM32F4T<devices::ScbViaSTM32F4>;
                using PllCfg    = ::stm32::f4::PllCfgT<::stm32::f4::f407::PllCfgValidCheck>;
                using Pwr       = ::stm32::f4::Pwr;
                using Rcc       = RccT<PllCfg, Flash, Pwr>;
                using Scb       = ::devices::ScbViaSTM32F4;

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f4::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using D     = ::stm32::GpioT<Rcc, GPIOD_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using E     = ::stm32::GpioT<Rcc, GPIOE_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using F     = ::stm32::GpioT<Rcc, GPIOF_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using G     = ::stm32::GpioT<Rcc, GPIOG_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using H     = ::stm32::GpioT<Rcc, GPIOH_BASE, ::stm32::f4::GpioPinConfiguration>;
                }; /* struct gpio */

                struct Uart {
                    template<typename PinT> using Usart1 = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart2 = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart3 = ::stm32::UartT<Rcc, USART3_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Uart4  = ::stm32::UartT<Rcc, UART4_BASE,  stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Uart5  = ::stm32::UartT<Rcc, UART5_BASE,  stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart6 = ::stm32::UartT<Rcc, USART6_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                }; /* struct Uart */
            }; /* struct Cpu */
        } /* namespace f407 */
    } /* namespace f4 */

    using Cpu = f4::f407::Cpu;

    using Gpio = Cpu::Gpio;
    using GpioEngine = Cpu::Gpio::Engine;
    using Rcc = Cpu::Rcc;
    using Uart = Cpu::Uart;

    MAP_RCC_ENGINE(GPIOA);
    MAP_RCC_ENGINE(GPIOB);
    MAP_RCC_ENGINE(GPIOC);
    MAP_RCC_ENGINE(GPIOD);
    MAP_RCC_ENGINE(GPIOE);
    MAP_RCC_ENGINE(GPIOF);
    MAP_RCC_ENGINE(GPIOG);
    MAP_RCC_ENGINE(GPIOH);

    MAP_RCC_ENGINE(USART1);
    MAP_RCC_ENGINE(USART2);
    MAP_RCC_ENGINE(USART3);
    MAP_RCC_ENGINE(UART4);
    MAP_RCC_ENGINE(UART5);
    MAP_RCC_ENGINE(USART6);
} /* namespace stm32 */

#endif /* _STM32_CPU_HPP_F936BB8E_9326_416B_BD61_9F6F683DFF3D */
