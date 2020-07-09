/*-
 * $Copyright$
-*/

#ifndef __UART_ACCESS_STM32F4_77f9b55a_3043_4d0e_96e9_67bf28aef0a5
#define __UART_ACCESS_STM32F4_77f9b55a_3043_4d0e_96e9_67bf28aef0a5

#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/RccViaSTM32.hpp>
#include <gpio/GpioPin.hpp>

namespace uart {

/*******************************************************************************
 *
 ******************************************************************************/
class UartAccessViaSTM32F4 {
public:
    typedef enum Uart_e {
        e_USART1,
        e_USART2,
        e_USART3,
        e_UART4,
        e_UART5,
        e_USART6
    } Uart_t;

    typedef enum UartFlowControl_e {
        e_FlowControl_None,
    } UartFlowControl_t;

    typedef enum UartWordLength_e {
        e_WordLength_8 = 0,
        e_WordLength_9 = 1,
    } UartWordLength_t;

    typedef enum UartParity_e {
        e_Parity_None,
        e_Parity_Even,
        e_Parity_Odd,
    } UartParity_t;
    
    typedef enum UartStopBits_e {
        e_StopBits_1    = 0x0,
        e_StopBits_0_5  = 0x1,
        e_StopBits_2    = 0x2,
        e_StopBits_1_5  = 0x3,
    } UartStopBits_t;

    typedef enum UartBaudRate_e {
        e_BaudRate_9600     = 9600,
        e_BaudRate_19200    = 19200,
        e_BaudRate_38400    = 38400,
        e_BaudRate_57600    = 57600,
        e_BaudRate_115200   = 115200,
        e_BaudRate_230400   = 230400,
    } UartBaudRate_t;

    UartAccessViaSTM32F4(USART_TypeDef * const p_uart);
    ~UartAccessViaSTM32F4();

    void putf(const char p_char) const;
    static void putf(void *p_this, const char p_char);

    void setFlowControl(const UartFlowControl_t p_flowControl) const;
    void setup(const UartWordLength_t p_bits, const UartParity_t p_parity, const UartStopBits_t p_stopBits) const;

protected:
    void initialize(void) const;
    void terminate(void) const;

    void setBaudRate(const UartBaudRate_t p_baudRate, const unsigned p_clk) const;

private:
    void enable(void) const;
    void disable(void) const;
    
    USART_TypeDef * const   m_uart;
};


/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t> struct UartAccessViaSTM32F4Helper;

#if defined(USART1_BASE)
template<> struct UartAccessViaSTM32F4Helper<USART1_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart1;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Usart1;
};
#endif
#if defined(USART2_BASE)
template<> struct UartAccessViaSTM32F4Helper<USART2_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart2;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Usart2;
};
#endif
#if defined(USART3_BASE)
template<> struct UartAccessViaSTM32F4Helper<USART3_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart3;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Usart3;
};
#endif
#if defined(UART4_BASE)
template<> struct UartAccessViaSTM32F4Helper<UART4_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart4;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Uart4;
};
#endif
#if defined(UART5_BASE)
template<> struct UartAccessViaSTM32F4Helper<UART5_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart5;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Uart5;
};
#endif
#if defined(USART6_BASE)
template<> struct UartAccessViaSTM32F4Helper<USART6_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_gpioFunction = gpio::GpioAccessViaSTM32F4::e_Uart6;
    static const auto m_rcc = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Usart6;
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
typedef gpio::PinT< gpio::EngineT< gpio::GpioAccessViaSTM32F4 > > DefaultPin_t;

template<intptr_t UartT, typename PinT = DefaultPin_t, typename RccT = devices::RccViaSTM32F4>
class UartAccessViaSTM32F4PinT : public UartAccessViaSTM32F4 {
public:
    UartAccessViaSTM32F4PinT(RccT &p_rcc, PinT &p_rx, PinT &p_tx)
      : UartAccessViaSTM32F4(reinterpret_cast<USART_TypeDef *>(UartT)), m_rcc(p_rcc), m_rx(p_rx), m_tx(p_tx) {
        m_rcc.enable(UartAccessViaSTM32F4Helper<UartT>::m_rcc);
        m_rx.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_PullUp, UartAccessViaSTM32F4Helper<UartT>::m_gpioFunction);
        m_tx.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_PullUp, UartAccessViaSTM32F4Helper<UartT>::m_gpioFunction);
        this->initialize();
    }

    ~UartAccessViaSTM32F4PinT(void) {
        this->terminate();
        m_tx.disable();
        m_rx.disable();
        m_rcc.disable(UartAccessViaSTM32F4Helper<UartT>::m_rcc);
    }

    void setBaudRate(const UartBaudRate_t p_baudRate) {
        UartAccessViaSTM32F4::setBaudRate(p_baudRate, m_rcc.getClockSpeed(UartAccessViaSTM32F4Helper<UartT>::m_rcc));
    }

private:
    RccT &m_rcc;
    PinT &m_rx;
    PinT &m_tx;
};

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(USART1_BASE)
typedef UartAccessViaSTM32F4PinT<USART1_BASE> UartAccessSTM32F4_Uart1;
#endif
#if defined(USART2_BASE)
typedef UartAccessViaSTM32F4PinT<USART2_BASE> UartAccessSTM32F4_Uart2;
#endif
#if defined(USART3_BASE)
typedef UartAccessViaSTM32F4PinT<USART3_BASE> UartAccessSTM32F4_Uart3;
#endif
#if defined(UART4_BASE)
typedef UartAccessViaSTM32F4PinT<UART4_BASE> UartAccessSTM32F4_Uart4;
#endif
#if defined(UART5_BASE)
typedef UartAccessViaSTM32F4PinT<UART5_BASE> UartAccessSTM32F4_Uart5;
#endif
#if defined(USART6_BASE)
typedef UartAccessViaSTM32F4PinT<USART6_BASE> UartAccessSTM32F4_Uart6;
#endif

}; /* namespace uart */

#endif /* __UART_LINUX_HPP_8f0c0c66_5e56_431b_8e1a_47bc71d178c8 */
