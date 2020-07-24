/*-
 * $Copyright$
 */
#ifndef _UART_HPP_879428FD_288E_4A02_A5BC_43EA24A5831E
#define _UART_HPP_879428FD_288E_4A02_A5BC_43EA24A5831E

#include <stm32/Engine.hpp>
#include <stm32f4xx.h>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
class UartEngine {
    USART_TypeDef & m_uart;

public:
    enum class FlowControl_e {
        e_None,
    };

    enum class WordLength_e {
        e_8 = 0,
        e_9 = 1,
    };

    enum class Parity_e {
        e_None,
        e_Even,
        e_Odd,
    };

    enum class StopBits_e {
        e_1     = 0x0,
        e_0_5   = 0x1,
        e_2     = 0x2,
        e_1_5   = 0x3,
    };

    enum class BaudRate_e {
        e_9600      = 9600,
        e_19200     = 19200,
        e_38400     = 38400,
        e_57600     = 57600,
        e_115200    = 115200,
        e_230400    = 230400,
    };

private:
    /* TODO Flow-control != e_FlowControl_None is not implemented */
    void
    setFlowControl(const enum FlowControl_e p_flowControl) const {
        switch(p_flowControl) {
        case FlowControl_e::e_None:
            m_uart.CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
            break;
        }
    }

    void
    enable(void) const {
        m_uart.CR1 |= USART_CR1_UE;
    }

    void
    disable(void) const {
        m_uart.CR1 &= ~USART_CR1_UE;
    }

protected:
    UartEngine(USART_TypeDef * const p_uart) : m_uart(*p_uart) {

    };

    ~UartEngine() = default;

    void
    initialize(void) const {
        m_uart.CR1 |= (USART_CR1_TE | USART_CR1_RE);

        setFlowControl(FlowControl_e::e_None);
        setup(WordLength_e::e_8, Parity_e::e_None, StopBits_e::e_1);
        enable();
    }

    void
    terminate(void) const {
        disable();
    }

public:
    /* FIXME Make a Policy; depends on CPU */
    void
    putf(const char p_char) const {
        while (!(m_uart.SR & USART_SR_TXE));

        m_uart.DR = p_char;

        while (!(m_uart.SR & USART_SR_TC));
    }

    static void
    putf(void *p_this, const char p_char) {
        const UartEngine *obj = reinterpret_cast<const UartEngine *>(p_this);
        obj->putf(p_char);
    }

    void
    setup(WordLength_e p_bits, Parity_e p_parity, StopBits_e p_stopBits) const {
        m_uart.CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
        m_uart.CR1 |= static_cast<unsigned>(p_bits) << USART_CR1_M_Pos; /* FIXME 7 Bit Operation won't work with this */

        switch (p_parity) {
        case Parity_e::e_None:
            break;
        case Parity_e::e_Even:
            m_uart.CR1 |= USART_CR1_PCE;
            break;
        case Parity_e::e_Odd:
            m_uart.CR1 |= (USART_CR1_PCE | USART_CR1_PS);
            break;
        }

        m_uart.CR2 &= ~(USART_CR2_STOP);
        m_uart.CR2 |= static_cast<unsigned>(p_stopBits) << USART_CR2_STOP_Pos;
    }

    template<typename BrrPolicyT>
    void
    setBaudRate(const BrrPolicyT &, UartEngine::BaudRate_e p_baudRate, const unsigned p_clk) {
        BrrPolicyT::setBaudRate(m_uart, p_baudRate, p_clk);
    }
}; /* class UartEngine */
/*****************************************************************************/

/*****************************************************************************/
class BrrPolicyNoOversampling {
    static
    unsigned
    calculateBrr(UartEngine::BaudRate_e p_baudRate, unsigned p_clk) {
        /*
        * Baud Rate is calculated as:
        *
        * f_ck / (8 * (2 - OVER8) * USARTDIV)
        *
        * so the USARTDIV needs to be:
        *
        *  USARTDIV = f_ck / (8 * (2 - OVER8) * p_baudRate)
        */
        const unsigned integerdivider = (25 * p_clk) / (4 * static_cast<unsigned>(p_baudRate));
        unsigned brr = (integerdivider / 100) << 4;

        const unsigned fractionaldivider = integerdivider - (100 * (brr >> 4));
        brr |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t) 0x0F);

        return (brr);
    }

public:
    template<typename UsartT>
    static void
    setBaudRate(UsartT &p_uart, UartEngine::BaudRate_e p_baudRate, const unsigned p_clk) {
        p_uart.BRR = BrrPolicyNoOversampling::calculateBrr(p_baudRate, p_clk);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class BrrPolicyWithOversampling : public BrrPolicyNoOversampling {
    static
    unsigned
    calculateBrr(UartEngine::BaudRate_e p_baudRate, unsigned p_clk) {
       /*
        * Baud Rate is calculated as:
        *
        * f_ck / (8 * (2 - OVER8) * USARTDIV)
        *
        * so the USARTDIV needs to be:
        *
        *  USARTDIV = f_ck / (8 * (2 - OVER8) * p_baudRate)
        */
        const unsigned integerdivider = (25 * p_clk) / (2 * static_cast<unsigned>(p_baudRate));
        unsigned brr = (integerdivider / 100) << 4;

        const unsigned fractionaldivider = integerdivider - (100 * (brr >> 4));
        brr |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t) 0x0F);

        return (brr);
    }

public:
    template<typename UsartT>
    static void
    setBaudRate(const UsartT &p_uart, UartEngine::BaudRate_e p_baudRate, const unsigned p_clk) {
        /* TODO This should be using macro USART_CR1_OVER8 */
        if (p_uart.CR1 & (1 << 15)) {
            p_uart.BRR = BrrPolicyWithOversampling::calculateBrr(p_baudRate, p_clk);
        } else {
            BrrPolicyNoOversampling::setBaudRate(p_uart, p_baudRate, p_clk);
        }
    }
};
/*****************************************************************************/

/*****************************************************************************/
template<
    typename RccT,
    intptr_t Address,
    typename BrrPolicyT,
    typename PinT
>
class UartT : public EngineT<Address>, public UartEngine, BrrPolicyT {
    const RccT &m_rcc;

public:
    constexpr UartT(const RccT &p_rcc, const PinT &p_tx, const PinT &p_rx)
      : UartEngine(reinterpret_cast<USART_TypeDef *>(Address)), m_rcc(p_rcc) {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

        p_tx.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_rx.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));

        this->initialize();

        /*
         * Set Baud Rate to some default value here.
         *
         * Some code may need to use the UART before setBaudRate() is called from an application.
         * 
         * This is the case for the Init Code in the USB Stack, if the setBaudRate() would otherwise
         * be called in the firmware's main() function.
         */
        this->setBaudRate(UartEngine::BaudRate_e::e_9600);
    }

    ~UartT() {
        this->terminate();

        // m_tx.disable();
        // m_rx.disable();

        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    }

    void
    setBaudRate(UartEngine::BaudRate_e p_baudRate) {
        UartEngine::setBaudRate(BrrPolicyNoOversampling(), p_baudRate, m_rcc.getEngineClockSpeed(* static_cast<EngineT<Address> *>(this)));
    }
}; /* class UartT */

/*
 * Assert that class type really is just the size of two pointers, i.e. that
 * the base class EngineT<> does not add to the size of the object
 */
struct Dummy_s { };
static_assert(sizeof(UartT<void *, 0, Dummy_s, Dummy_s>) == 2 * sizeof(void *));
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _UART_HPP_879428FD_288E_4A02_A5BC_43EA24A5831E */
