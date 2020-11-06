/*-
 * $Copyright$
-*/

#ifndef __USB_MOUSE_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1
#define __USB_MOUSE_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1

#include <tasks/Task.hpp>

#include <gpio/GpioPin.hpp>
#include <usb/UsbApplication.hpp>

#include <phisch/log.h>

#include <FreeRTOS.h>
#include <FreeRTOS/include/queue.h>
#include <FreeRTOS/include/semphr.h>

namespace tasks {

/***************************************************************************//**
 * @brief USB Mouse Button Handler.
 *
 * This Task reacts on events from a Queue and translates them to Button press /
 * release in the USB Mouse Application.
 *******************************************************************************/
template<
    typename UsbMouseApplicationT
>
class UsbMouseButtonHandlerT : public Task {
private:
    UsbMouseApplicationT &      m_usbMouseApplication;
    const gpio::GpioPin *       m_led;
    QueueHandle_t               m_buttonHandlerQueue;
    SemaphoreHandle_t           m_usbMutex;

    void run(void) override {
        bool buttonState;

        while(xQueueReceive(this->m_buttonHandlerQueue, &buttonState, portMAX_DELAY) == pdTRUE) {
            if (m_led != nullptr) {
                m_led->set(buttonState);
            } else {
                PHISCH_LOG("UsbMouseButtonHandler::%s(): Button State = 0x%x\r\n", __func__, buttonState);
            }

            m_usbMouseApplication.setButton(UsbMouseApplicationT::Button_e::e_LeftButton, buttonState);

            xSemaphoreTake(m_usbMutex, portMAX_DELAY);
            m_usbMouseApplication.updateHost();
            xSemaphoreGive(m_usbMutex);
        }

        PHISCH_LOG("UsbMouseButtonHandler::%s(): Failed to receive Button State from queue.\r\n", __func__);
        m_led->disable();

        assert(false);
    };

public:
    UsbMouseButtonHandlerT(const char * const p_name, const unsigned p_priority, UsbMouseApplicationT &p_usbMouseApplication)
      : Task(p_name, p_priority), m_usbMouseApplication(p_usbMouseApplication), m_led(nullptr) {

    }

    UsbMouseButtonHandlerT(const char * const p_name, const unsigned p_priority, UsbMouseApplicationT &p_usbMouseApplication, const gpio::GpioPin * const p_led)
      : Task(p_name, p_priority), m_usbMouseApplication(p_usbMouseApplication), m_led(p_led) {

    }

    virtual ~UsbMouseButtonHandlerT() = default;

    void setRxQueue(const QueueHandle_t p_buttonHandlerQueue) {
        this->m_buttonHandlerQueue = p_buttonHandlerQueue;
    }

    void
    setUsbMutex(SemaphoreHandle_t p_usbMutex) {
        m_usbMutex = p_usbMutex;
    }
};

/***************************************************************************//**
 * @brief USB Mouse Mover.
 *
 * This Task updates the x- and y-Position periodically.
 *******************************************************************************/
template<
    typename UsbMouseApplicationT
>
class UsbMouseMoverT : public PeriodicTask {
private:
    UsbMouseApplicationT &      m_usbMouseApplication;
    const gpio::GpioPin * const m_led;
    int                         m_xOffset;
    int                         m_yOffset;
    int                         m_direction;
    SemaphoreHandle_t           m_usbMutex;

    int executePeriod(void) override {
        m_direction *= -1;

        if (m_led != nullptr) {
            m_led->set(m_direction > 0);
        }

        m_usbMouseApplication.setXAxis(m_direction * m_xOffset);
        m_usbMouseApplication.setYAxis(m_direction * m_yOffset);

        xSemaphoreTake(m_usbMutex, portMAX_DELAY);
        m_usbMouseApplication.updateHost();
        xSemaphoreGive(m_usbMutex);

        return (0);
    }

public:
    UsbMouseMoverT(const char * const p_name, const unsigned p_priority, const unsigned p_periodMs,
      UsbMouseApplicationT &p_usbMouseApplication, const int p_xOffset, const int p_yOffset, const gpio::GpioPin *p_led = nullptr)
        : PeriodicTask(p_name, p_priority, p_periodMs, 256), m_usbMouseApplication(p_usbMouseApplication), m_led(p_led),
            m_xOffset(p_xOffset), m_yOffset(p_yOffset), m_direction(1) {

    }

    void
    setXOffset(int p_xOffset) {
        m_xOffset = p_xOffset;
    }

    void
    setYOffset(int p_yOffset) {
        m_yOffset = p_yOffset;
    }

    void
    setUsbMutex(SemaphoreHandle_t p_usbMutex) {
        m_usbMutex = p_usbMutex;
    }
};

} /* namespace tasks */

#endif /* __USB_MOUSE_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1 */