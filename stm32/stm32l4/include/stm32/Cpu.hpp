/*-
 * $Copyright$
 */
#ifndef _STM32L4_CPU_HPP_B42B268F_D077_4C0A_BA07_05B4492A8FDB
#define _STM32L4_CPU_HPP_B42B268F_D077_4C0A_BA07_05B4492A8FDB

#include <stm32f4xx.h>

#include <stm32/RccEngine.hpp>

#include <stm32f4/FlashViaSTM32F4.hpp>
#include <stm32/Gpio.hpp>
#include <stm32f4/GpioPinConfiguration.hpp>
#include <stm32/Nvic.hpp>
#include <stm32l4/PllCfg.hpp>
#include <stm32f4/PwrViaSTM32F4.hpp>
#include <stm32l4/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Uart.hpp>

namespace stm32 {
    using Nvic      = NvicT<Scb>;

    namespace l4 {
        namespace l432 {
            struct Cpu {
                using Flash     = ::stm32::f4::Flash;
                using PllCfg    = ::stm32::l4::PllCfgT<::stm32::l4::l432::PllCfgValidCheck, 16, 32>;
                using Pwr       = ::stm32::f4::Pwr;
                using Rcc       = RccT<PllCfg, Flash, Pwr>;

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f4::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using H     = ::stm32::GpioT<Rcc, GPIOH_BASE, ::stm32::f4::GpioPinConfiguration>;
                }; /* struct gpio */

                struct Uart {
                    template<typename PinT> using Usart1 = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart2 = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                }; /* struct Uart */
            }; /* struct Cpu */
        } /* namespace l432 */
    } /* namespace l4 */

#if defined(STM32L432xx)
    using Cpu = l4::l432::Cpu;
#endif

    using Gpio = Cpu::Gpio;
    using GpioEngine = Cpu::Gpio::Engine;
    using Rcc = Cpu::Rcc;
    using Uart = Cpu::Uart;

#if defined(GPIOA_BASE)
    MAP_RCC_ENGINE(GPIOA);
#endif
#if defined(GPIOB_BASE)
    MAP_RCC_ENGINE(GPIOB);
#endif
#if defined(GPIOC_BASE)
    MAP_RCC_ENGINE(GPIOC);
#endif
#if defined(GPIOD_BASE)
    MAP_RCC_ENGINE(GPIOD);
#endif
#if defined(GPIOE_BASE)
    MAP_RCC_ENGINE(GPIOE);
#endif
#if defined(GPIOF_BASE)
    MAP_RCC_ENGINE(GPIOF);
#endif
#if defined(GPIOG_BASE)
    MAP_RCC_ENGINE(GPIOG);
#endif
#if defined(GPIOH_BASE)
    MAP_RCC_ENGINE(GPIOH);
#endif

#if defined(USART1_BASE)
    MAP_RCC_ENGINE(USART1);
#endif
#if defined(USART2_BASE)
    MAP_RCC_ENGINE(USART2);
#endif
#if defined(USART3_BASE)
    MAP_RCC_ENGINE(USART3);
#endif
#if defined(UART4_BASE)
    MAP_RCC_ENGINE(UART4);
#endif
#if defined(UART5_BASE)
    MAP_RCC_ENGINE(UART5);
#endif
#if defined(USART6_BASE)
    MAP_RCC_ENGINE(USART6);
#endif
} /* namespace stm32 */

#endif /*  _STM32L4_CPU_HPP_B42B268F_D077_4C0A_BA07_05B4492A8FDB */
