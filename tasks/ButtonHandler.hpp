/*-
 * $Copyright$
-*/

#ifndef __BUTTON_HANDLER_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1
#define __BUTTON_HANDLER_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1

#include <tasks/Task.hpp>

#include <gpio/GpioPin.hpp>
#include <usb/UsbInEndpoint.hpp>

#include <phisch/log.h>

namespace tasks {

class ButtonHandler : public Task {
private:
    const usb::UsbIrqInEndpoint &   m_usbIrqInEndpoint;
    const gpio::GpioPin &           m_led;
    QueueHandle_t                   m_buttonHandlerQueue;

    void run(void) override {
        bool buttonState;
        uint8_t data[3] = { 0, 0, 0 };

        while(xQueueReceive(this->m_buttonHandlerQueue, &buttonState, portMAX_DELAY) == pdTRUE) {
            if (buttonState) {
                data[0] |= 0x1;
                m_led.set(gpio::GpioPin::On);
            } else {
                data[0] &= ~0x1;
                m_led.set(gpio::GpioPin::Off);
            }

            if (m_usbIrqInEndpoint.isEnabled()) {
                m_usbIrqInEndpoint.write(data, sizeof(data));
            }
        }
        PHISCH_LOG("ButtonHandler::%s(): Failed to receive Button State from queue.\r\n", __func__);
        m_led.set(gpio::GpioPin::HiZ);
        assert(false);
    };

public:
    ButtonHandler(const char * const p_name, const unsigned p_priority, const usb::UsbIrqInEndpoint &p_usbIrqInEndpoint, const gpio::GpioPin &p_led)
      : Task(p_name, p_priority), m_usbIrqInEndpoint(p_usbIrqInEndpoint), m_led(p_led) {

    }

    virtual ~ButtonHandler() {

    };

    void setRxQueue(const QueueHandle_t p_buttonHandlerQueue) {
        this->m_buttonHandlerQueue = p_buttonHandlerQueue;
    }
};

} /* namespace tasks */

#endif /* __BUTTON_HANDLER_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1 */