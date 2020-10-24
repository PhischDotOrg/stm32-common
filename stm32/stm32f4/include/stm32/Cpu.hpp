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
#include <stm32/Nvic.hpp>
#include <stm32f4/PllCfg.hpp>
#include <stm32f4/PwrViaSTM32F4.hpp>
#include <stm32f4/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Spi.hpp>
#include <stm32/Uart.hpp>

#include <stm32f4/dma/Engine.hpp>
#include <stm32f4/dma/Stream.hpp>
#include <stm32f4/dma/Channel.hpp>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/
    using Nvic      = NvicT<Scb>;

    namespace f4 {
#if defined(STM32F401xC)
        namespace f401 {
            struct Cpu {
                using Flash     = ::stm32::f4::Flash;
                using PllCfg    = ::stm32::f4::PllCfgT<8, ::stm32::f4::f407::PllCfgValidCheck>;
                using Pwr       = ::stm32::f4::Pwr;
                using Rcc       = RccT<PllCfg, Flash, Pwr>;

                struct Dma {
                    using Engine1   = ::stm32::dma::DmaEngineT<Rcc, DMA1_BASE>;
                    using Engine2   = ::stm32::dma::DmaEngineT<Rcc, DMA2_BASE>;
                }; /* struct Dma */

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f4::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using D     = ::stm32::GpioT<Rcc, GPIOD_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using E     = ::stm32::GpioT<Rcc, GPIOE_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using H     = ::stm32::GpioT<Rcc, GPIOH_BASE, ::stm32::f4::GpioPinConfiguration>;
                }; /* struct gpio */

                struct Spi {
                    // template<
                    //     typename PinT,
                    //     typename DmaChannelTxT,
                    //     typename DmaChannelRxT
                    // > using DmaSpi1 = ::stm32::f4::SpiViaDmaT<
                    //     SPI1_BASE,
                    //     Rcc,
                    //     DmaChannelTxT,
                    //     DmaChannelRxT,
                    //     PinT
                    // >;

                    template<
                        typename PinT
                    > using Spi1 = ::stm32::SpiT<
                        SPI1_BASE,
                        Rcc,
                        PinT
                    >;
                }; /* struct Spi */

                struct Uart {
                    template<typename PinT> using Usart1 = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart2 = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart6 = ::stm32::UartT<Rcc, USART6_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                }; /* struct Uart */
            }; /* struct Cpu */
        } /* namespace f407 */
#endif /* defined(STM32F407xx) */
#if defined(STM32F407xx)
        namespace f407 {
            struct Cpu {
                using Flash     = ::stm32::f4::Flash;
                using PllCfg    = ::stm32::f4::PllCfgT<8, ::stm32::f4::f407::PllCfgValidCheck>;
                using Pwr       = ::stm32::f4::Pwr;
                using Rcc       = RccT<PllCfg, Flash, Pwr>;

                struct Dma {
                    using Engine1   = ::stm32::dma::DmaEngineT<Rcc, DMA1_BASE>;
                    using Engine2   = ::stm32::dma::DmaEngineT<Rcc, DMA2_BASE>;
                }; /* struct Dma */

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

                struct Spi {
                    // template<
                    //     typename PinT,
                    //     typename DmaChannelTxT,
                    //     typename DmaChannelRxT
                    // > using DmaSpi1 = ::stm32::f4::SpiViaDmaT<
                    //     SPI1_BASE,
                    //     Rcc,
                    //     DmaChannelTxT,
                    //     DmaChannelRxT,
                    //     PinT
                    // >;

                    template<
                        typename PinT
                    > using Spi1 = ::stm32::SpiT<
                        SPI1_BASE,
                        Rcc,
                        PinT
                    >;
                }; /* struct Spi */

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
#endif /* defined(STM32F407xx) */
#if defined(STM32F411xE)
        namespace f411 {
            struct Cpu {
                using Flash     = ::stm32::f4::Flash;
                using PllCfg    = ::stm32::f4::PllCfgT<16, ::stm32::f4::f411::PllCfgValidCheck>;
                using Pwr       = ::stm32::f4::Pwr;
                using Rcc       = RccT<PllCfg, Flash, Pwr>;

                struct Dma {
                    using Engine1   = ::stm32::dma::DmaEngineT<Rcc, DMA1_BASE>;
                    using Engine2   = ::stm32::dma::DmaEngineT<Rcc, DMA2_BASE>;
                }; /* struct Dma */

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f4::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using D     = ::stm32::GpioT<Rcc, GPIOD_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using E     = ::stm32::GpioT<Rcc, GPIOE_BASE, ::stm32::f4::GpioPinConfiguration>;
                    using H     = ::stm32::GpioT<Rcc, GPIOH_BASE, ::stm32::f4::GpioPinConfiguration>;
                }; /* struct Gpio */

                struct Spi {
                    // template<
                    //     typename PinT,
                    //     typename DmaChannelTxT,
                    //     typename DmaChannelRxT
                    // > using DmaSpi1 = ::stm32::f4::SpiViaDmaT<
                    //     SPI1_BASE,
                    //     Rcc,
                    //     DmaChannelTxT,
                    //     DmaChannelRxT,
                    //     PinT
                    // >;

                    template<
                        typename PinT
                    > using Spi1 = ::stm32::SpiT< SPI1_BASE, Rcc, PinT >;

                    template<
                        typename PinT
                    > using Spi2 = ::stm32::SpiT< SPI2_BASE, Rcc, PinT >;

                    template<
                        typename PinT
                    > using Spi3 = ::stm32::SpiT< SPI3_BASE, Rcc, PinT >;

                    template<
                        typename PinT
                    > using Spi4 = ::stm32::SpiT< SPI4_BASE, Rcc, PinT >;
                }; /* struct Spi */

                struct Uart {
                    template<typename PinT> using Usart1 = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart2 = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                    template<typename PinT> using Usart6 = ::stm32::UartT<Rcc, USART6_BASE, stm32::BrrPolicyWithOversampling, PinT>;
                }; /* struct Uart */
            }; /* struct Cpu */
        } /* namespace f411 */
#endif /* defined(STM32F411xE) */
    } /* namespace f4 */

#if defined(STM32F401xC)
    using Cpu = f4::f401::Cpu;
#endif
#if defined(STM32F407xx)
    using Cpu = f4::f407::Cpu;
#endif
#if defined(STM32F411xE)
    using Cpu = f4::f411::Cpu;
#endif

    using Dma = Cpu::Dma;
    using Gpio = Cpu::Gpio;
    using GpioEngine = Cpu::Gpio::Engine;
    using Rcc = Cpu::Rcc;
    using Spi = Cpu::Spi;
    using Uart = Cpu::Uart;

#if defined(DMA1_BASE)
    MAP_RCC_ENGINE(DMA1);
#endif
#if defined(DMA2_BASE)
    MAP_RCC_ENGINE(DMA2);
#endif

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

#if defined(SPI1_BASE)
    MAP_RCC_ENGINE(SPI1);
#endif
#if defined(SPI2_BASE)
    MAP_RCC_ENGINE(SPI2);
#endif
#if defined(SPI3_BASE)
    MAP_RCC_ENGINE(SPI3);
#endif
/* FIXME Are the engines really not present? */
// #if defined(SPI4_BASE)
//     MAP_RCC_ENGINE(SPI4);
// #endif
// #if defined(SPI5_BASE)
//     MAP_RCC_ENGINE(SPI5);
// #endif

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
/*****************************************************************************/

/*****************************************************************************/
#if defined(DMA1_Stream0_BASE)
    MAP_NVIC_IRQ(DMA1_Stream0);
#endif
#if defined(DMA1_Stream1_BASE)
    MAP_NVIC_IRQ(DMA1_Stream1);
#endif
#if defined(DMA1_Stream2_BASE)
    MAP_NVIC_IRQ(DMA1_Stream2);
#endif
#if defined(DMA1_Stream3_BASE)
    MAP_NVIC_IRQ(DMA1_Stream3);
#endif
#if defined(DMA1_Stream4_BASE)
    MAP_NVIC_IRQ(DMA1_Stream4);
#endif
#if defined(DMA1_Stream5_BASE)
    MAP_NVIC_IRQ(DMA1_Stream5);
#endif
#if defined(DMA1_Stream6_BASE)
    MAP_NVIC_IRQ(DMA1_Stream6);
#endif
#if defined(DMA1_Stream7_BASE)
    MAP_NVIC_IRQ(DMA1_Stream7);
#endif

#if defined(DMA2_Stream0_BASE)
    MAP_NVIC_IRQ(DMA2_Stream0);
#endif
#if defined(DMA2_Stream1_BASE)
    MAP_NVIC_IRQ(DMA2_Stream1);
#endif
#if defined(DMA2_Stream2_BASE)
    MAP_NVIC_IRQ(DMA2_Stream2);
#endif
#if defined(DMA2_Stream3_BASE)
    MAP_NVIC_IRQ(DMA2_Stream3);
#endif
#if defined(DMA2_Stream4_BASE)
    MAP_NVIC_IRQ(DMA2_Stream4);
#endif
#if defined(DMA2_Stream5_BASE)
    MAP_NVIC_IRQ(DMA2_Stream5);
#endif
#if defined(DMA2_Stream6_BASE)
    MAP_NVIC_IRQ(DMA2_Stream6);
#endif
#if defined(DMA2_Stream7_BASE)
    MAP_NVIC_IRQ(DMA2_Stream7);
#endif
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_CPU_HPP_F936BB8E_9326_416B_BD61_9F6F683DFF3D */
