/*-
 * $Copyright$
-*/

#include <tasks/Button.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"

#if defined(__cplusplus)
} /* extern "C"*/
#endif /* defined(__cplusplus) */

namespace tasks {

const char * const Button::m_name = "button";

/*******************************************************************************
 *
 ******************************************************************************/
Button::Button(uart::UartDevice *p_uart, gpio::GpioPin *p_led)
  : InterruptTask(NULL, m_name), m_uart(p_uart), m_led(p_led), m_status(false) {

}

/*******************************************************************************
 *
 ******************************************************************************/
Button::~Button() {

}

/*******************************************************************************
 *
 ******************************************************************************/
void
Button::handle_irq() {
    this->m_status = !this->m_status;
    this->m_led->set(this->m_status ? gpio::On : gpio::Off);
    this->m_uart->printf("Button::handle_irq(m_status=%d)\r\n", this->m_status);
}

}
