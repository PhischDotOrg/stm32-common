/*-
 * $Copyright$
-*/

#include <uart/UartAccessViaSTM32F4.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stddef.h>

namespace uart {

/*******************************************************************************
 *
 ******************************************************************************/
UartAccessViaSTM32F4::UartAccessViaSTM32F4(USART_TypeDef * const p_uart)
  : m_uart(p_uart) {
}

/*******************************************************************************
 *
 ******************************************************************************/
UartAccessViaSTM32F4::~UartAccessViaSTM32F4() {
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::initialize(void) const {
    this->m_uart->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    
    this->setFlowControl(e_FlowControl_None);
    this->setup(e_WordLength_8, e_Parity_None, e_StopBits_1);
    
    this->enable();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::setFlowControl(const UartFlowControl_t p_flowControl) const {
    switch(p_flowControl) {
    case e_FlowControl_None:
        this->m_uart->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
        break;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::setup(const UartWordLength_t p_bits, const UartParity_t p_parity, const UartStopBits_t p_stopBits) const {
    this->m_uart->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
    this->m_uart->CR1 |= p_bits << 12;

    switch (p_parity) {
    case e_Parity_None:
        break;
    case e_Parity_Even:
        this->m_uart->CR1 |= USART_CR1_PCE;
        break;
    case e_Parity_Odd:
        this->m_uart->CR1 |= (USART_CR1_PCE | USART_CR1_PS);
        break;
    }
    
    this->m_uart->CR2 &= ~(USART_CR2_STOP);
    this->m_uart->CR2 |= p_stopBits << 12;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::setBaudRate(const UartBaudRate_t p_baudRate, const unsigned p_clk) const {
    /*
     * Baud Rate is calculated as:
     * 
     * f_ck / (8 * (2 - OVER8) * USARTDIV)
     * 
     * so the USARTDIV needs to be:
     * 
     *  USARTDIV = f_ck / (8 * (2 - OVER8) * p_baudRate)
     */
    unsigned brr;
    if (this->m_uart->CR1 & USART_CR1_OVER8) {
        unsigned integerdivider = (25 * p_clk) / (2 * p_baudRate);
        brr = (integerdivider / 100) << 4;

        unsigned fractionaldivider = integerdivider - (100 * (brr >> 4));
        brr |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t) 0x0F);
    } else {
        unsigned integerdivider = (25 * p_clk) / (4 * p_baudRate);
        brr = (integerdivider / 100) << 4;

        unsigned fractionaldivider = integerdivider - (100 * (brr >> 4));
        brr |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t) 0x0F);
    }

    this->m_uart->BRR = brr & 0xFFFF;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::terminate(void) const {
    this->disable();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::enable(void) const {
    this->m_uart->CR1 |= USART_CR1_UE;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::disable(void) const {
    this->m_uart->CR1 &= ~USART_CR1_UE;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::putf(void *p_this, const char p_char) {
    UartAccessViaSTM32F4 *obj = reinterpret_cast<uart::UartAccessViaSTM32F4 *>(p_this);
    obj->putf(p_char);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UartAccessViaSTM32F4::putf(const char p_char) const {
    while (!(this->m_uart->SR & USART_SR_TXE));

    this->m_uart->DR = p_char;

    while (!(this->m_uart->SR & USART_SR_TC));
}

} /* namespace uart */
