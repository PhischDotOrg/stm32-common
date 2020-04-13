/*-
 * $Copyright$
-*/

#ifndef _GPIO_ACCESS_STM32_CPP_65550561_db94_4a13_af56_3603daee658a
#define _GPIO_ACCESS_STM32_CPP_65550561_db94_4a13_af56_3603daee658a

#include <gpio/GpioAccessViaSTM32F4.hpp>
#include <stm32f4/ScbViaSTM32F4.hpp>
#include <stm32f4/RccViaSTM32F4.hpp>

#include "stm32f4xx.h"

#include <errno.h>
#include <strings.h>

namespace gpio {

/*******************************************************************************
 * 
 ******************************************************************************/
GpioAccessViaSTM32F4::GpioAccessViaSTM32F4(GPIO_TypeDef * const p_gpio)
  : m_gpio(p_gpio) {
}

/*******************************************************************************
 *
 ******************************************************************************/
GpioAccessViaSTM32F4::~GpioAccessViaSTM32F4() {
}

/*******************************************************************************
 *
 ******************************************************************************/
int
GpioAccessViaSTM32F4::write(uint16_t p_value, uint16_t p_output, uint16_t p_mask) const {
    this->m_gpio->OTYPER &= ~(p_output & p_mask);
    this->m_gpio->OTYPER |= (~p_output & p_mask);
#if defined(__STM32F4xx_CMSIS_DEVICE_VERSION_MAIN) && defined(__STM32F4xx_CMSIS_DEVICE_VERSION_SUB1)
    #if (__STM32F4xx_CMSIS_DEVICE_VERSION_MAIN >= 2) && (__STM32F4xx_CMSIS_DEVICE_VERSION_SUB1 >= 3)
        #define USE_BSRR
    #else
        #error CMSIS Version defined, but too old for this code
    #endif /* CMSIS Version >= 2.3.x */
#elif defined(__STM32F4xx_CMSIS_VERSION_MAIN) && defined(__STM32F4xx_CMSIS_VERSION_SUB1)
    #if (__STM32F4xx_CMSIS_VERSION_MAIN >= 2) && (__STM32F4xx_CMSIS_VERSION_SUB1 >= 6)
        #define USE_BSRR
    #else
        #error CMSIS Version defined, but too old for this code
    #endif /* CMSIS Version >= 2.6.x */
#endif

#if defined(USE_BSRR)
    this->m_gpio->BSRR = (((~p_value & p_mask) << 16) & 0xFFFF0000)
                         | (((p_value & p_mask) << 0) & 0x0000FFFF);
#else
    this->m_gpio->BSRRH = (~p_value & p_mask);
    this->m_gpio->BSRRL = (p_value & p_mask);
#endif
    return (0);
}

/*******************************************************************************
 *
 ******************************************************************************/
int
GpioAccessViaSTM32F4::read(uint16_t &p_vector) const {
    p_vector = this->m_gpio->IDR;

    return (0);
}

/*******************************************************************************
 *
 ******************************************************************************/
int
GpioAccessViaSTM32F4::enable(const uint8_t p_pin,
                             const GpioAccessViaSTM32F4::Mode_e p_mode,
                             const GpioAccessViaSTM32F4::Termination_e p_termination,
                             const Function_e p_function) const {
    unsigned fnoffs = ((p_pin & 0x07) * 4);
    this->m_gpio->AFR[p_pin >> 3] &= ~(0xF << fnoffs);
    this->m_gpio->AFR[p_pin >> 3] |= p_function << fnoffs;

    unsigned offset = (p_pin * 2);
    this->m_gpio->MODER &= ~(0x3 << offset);
    this->m_gpio->MODER |= p_mode << offset;

    this->m_gpio->PUPDR &= ~(0x3 << offset);
    this->m_gpio->PUPDR |= p_termination << offset;
    
    this->m_gpio->OSPEEDR &= ~(0x3 << offset);
    this->m_gpio->OSPEEDR |= (0x2 << offset);

    return (0);
};

/*******************************************************************************
 *
 ******************************************************************************/
int
GpioAccessViaSTM32F4::disable(const uint8_t p_pin) const {
    unsigned offset = (p_pin * 2);
    this->m_gpio->MODER &= ~(0x3 << offset);
    this->m_gpio->PUPDR &= ~(0x3 << offset);

    unsigned fnoffs = (p_pin * 4);
    this->m_gpio->AFR[p_pin >> 3] &= ~(0xF << fnoffs);

    return (0);
}

}; /* namespace gpio */

#endif /* _GPIO_ACCESS_STM32_CPP_65550561_db94_4a13_af56_3603daee658a */
