/*-
 * $Copyright$
-*/

#ifndef __BUTTON_HPP_ce65a787_d7cd_4110_8321_b1dff94dd929
#define __BUTTON_HPP_ce65a787_d7cd_4110_8321_b1dff94dd929

#include <tasks/InterruptTask.hpp>

#include <uart/UartDevice.hpp>
#include <gpio/GpioPin.hpp>

namespace tasks {

class Button : public InterruptTask {
private:
    static const char * const   m_name;
    uart::UartDevice *          m_uart;
    gpio::GpioPin *             m_led;
    bool                        m_status;

public:
    Button(uart::UartDevice *p_uart, gpio::GpioPin *p_led);
    virtual ~Button();

    virtual void handle_irq(void);
};

} /* namespace tasks */

#endif /* __BUTTON_HPP_ce65a787_d7cd_4110_8321_b1dff94dd929 */