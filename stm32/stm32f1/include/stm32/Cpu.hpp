/*-
 * $Copyright$
 */
#ifndef _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A
#define _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A

#include <stm32f4xx.h>

#include <stm32f1/Flash.hpp>
#include <stm32/Gpio.hpp>
#include <stm32f1/GpioPinConfiguration.hpp>
#include <stm32/Nvic.hpp>
#include <stm32f1/PllCfg.hpp>
#include <stm32f1/Pwr.hpp>
#include <stm32f1/Rcc.hpp>
#include <stm32/Scb.hpp>
#include <stm32/Spi.hpp>
#include <stm32/Uart.hpp>

#include <stm32f1/dma/Engine.hpp>
#include <stm32f1/dma/Stream.hpp>
#include <stm32f1/dma/Channel.hpp>

#include <stm32f1/usb/Peripheral.hpp>
#include <stm32f1/usb/Device.hpp>
#include <stm32f1/usb/InEndpoint.hpp>
#include <stm32f1/usb/OutEndpoint.hpp>

namespace stm32 {
    using Nvic      = NvicT<Scb>;

    namespace f1 {
        namespace f103 {
            struct Cpu {
                using Flash     = f1::Flash;
                using PllCfg    = f1::PllCfg;
                using Rcc       = f1::Rcc;
                using Pwr       = f1::PwrT<Rcc, Scb>;

                struct Dma {
                    using Engine                                    = ::stm32::f1::DmaEngineT<Rcc, DMA1_BASE>;
                    template<unsigned nChannelNo> using Channel     = ::stm32::f1::DmaChannelT<Nvic, Engine, nChannelNo>;
                }; /* struct Dma */

                struct Gpio {
                    using Engine = ::stm32::GpioEngineT<stm32::f1::GpioPinConfiguration>;

                    using A     = ::stm32::GpioT<Rcc, GPIOA_BASE, stm32::f1::GpioPinConfiguration>;
                    using B     = ::stm32::GpioT<Rcc, GPIOB_BASE, stm32::f1::GpioPinConfiguration>;
                    using C     = ::stm32::GpioT<Rcc, GPIOC_BASE, stm32::f1::GpioPinConfiguration>;
                    using D     = ::stm32::GpioT<Rcc, GPIOD_BASE, stm32::f1::GpioPinConfiguration>;
                    using E     = ::stm32::GpioT<Rcc, GPIOE_BASE, stm32::f1::GpioPinConfiguration>;
                }; /* struct gpio */

                struct Spi {
                    // template<
                    //     typename PinT,
                    //     typename DmaChannelTxT,
                    //     typename DmaChannelRxT
                    // > using DmaSpi1 = ::stm32::f1::SpiViaDmaT<
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
                    template<typename PinT> using Usart1    = ::stm32::UartT<Rcc, USART1_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                    template<typename PinT> using Usart2    = ::stm32::UartT<Rcc, USART2_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                    template<typename PinT> using Usart3    = ::stm32::UartT<Rcc, USART3_BASE, stm32::BrrPolicyNoOversampling, PinT>;
                }; /* struct Uart */

                struct Usb {
                    using UsbDevice         = ::stm32::f1::usb::Device;

                    using CtrlInEndpoint    = ::stm32::f1::usb::CtrlInEndpoint;
                    using CtrlOutEndpoint   = ::stm32::f1::usb::CtrlOutEndpoint;

                    using BulkInEndpoint    = ::stm32::f1::usb::BulkInEndpoint;
                    using BulkOutEndpoint   = ::stm32::f1::usb::BulkOutEndpoint;

                    using IrqInEndpoint     = ::stm32::f1::usb::IrqInEndpoint;
                }; /* struct Usb */
            }; /* struct Cpu */
        } /* namespace f103 */
    } /* namespace f1 */

    using Cpu = f1::f103::Cpu;

    using Dma = Cpu::Dma;
    using Gpio = Cpu::Gpio;
    using GpioEngine = Cpu::Gpio::Engine;
    using Rcc = Cpu::Rcc;
    using Spi = Cpu::Spi;
    using Uart = Cpu::Uart;
    using Usb = Cpu::Usb;

    MAP_RCC_ENGINE(DMA1);

    MAP_RCC_ENGINE(GPIOA);
    MAP_RCC_ENGINE(GPIOB);
    MAP_RCC_ENGINE(GPIOC);
    MAP_RCC_ENGINE(GPIOD);

    MAP_RCC_ENGINE(PWR);

    MAP_RCC_ENGINE(SPI1);
    MAP_RCC_ENGINE(SPI2);

    MAP_RCC_ENGINE(TIM1);
    MAP_RCC_ENGINE(TIM2);
    MAP_RCC_ENGINE(TIM3);
    MAP_RCC_ENGINE(TIM4);

    MAP_RCC_ENGINE(USART1);
    MAP_RCC_ENGINE(USART2);
    MAP_RCC_ENGINE(USART3);

    MAP_RCC_ENGINE(USB);

    MAP_NVIC_IRQ(DMA1_Channel1);
    MAP_NVIC_IRQ(DMA1_Channel2);
    MAP_NVIC_IRQ(DMA1_Channel3);
    MAP_NVIC_IRQ(DMA1_Channel4);
    MAP_NVIC_IRQ(DMA1_Channel5);
    MAP_NVIC_IRQ(DMA1_Channel6);
    MAP_NVIC_IRQ(DMA1_Channel7);

    /* FIXME TIM1 has more than one IRQ handler */
    // MAP_NVIC_IRQ(TIM1);
    MAP_NVIC_IRQ(TIM2);
    MAP_NVIC_IRQ(TIM3);
    MAP_NVIC_IRQ(TIM4);

    // MAP_NVIC_IRQ(USB);
    template<> struct IrqTypeT< ::stm32::EngineT< (USB_BASE) > > {
        static constexpr auto m_irq = NvicBase::Irq_t::USB_LP_CAN1_RX0_IRQn;
    };

} /* namespace stm32 */

#endif /* _STM32_CPU_HPP_22D04FB3_F771_4620_A5A3_D3F32BBB711A */
