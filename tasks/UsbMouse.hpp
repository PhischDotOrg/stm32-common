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

#include <cmath>

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
    typename UsbMouseApplicationT,
    typename MovePolicyT
>
class UsbMouseMoverT : public PeriodicTask, MovePolicyT {
private:
    UsbMouseApplicationT &      m_usbMouseApplication;
    SemaphoreHandle_t           m_usbMutex;

    using Position_t = std::pair<int, int>;
    Position_t  m_oldPos;
    Position_t  m_newPos;

    int executePeriod(void) override {
        MovePolicyT::move();

        m_oldPos = m_newPos;
        m_newPos = MovePolicyT::getPosition();

        m_usbMouseApplication.setXAxis(m_newPos.first - m_oldPos.first);
        m_usbMouseApplication.setYAxis(m_newPos.second - m_oldPos.second);

        xSemaphoreTake(m_usbMutex, portMAX_DELAY);
        m_usbMouseApplication.updateHost();
        xSemaphoreGive(m_usbMutex);

        return (0);
    }

public:
    UsbMouseMoverT(const char * const p_name, const unsigned p_priority, const unsigned p_periodMs, UsbMouseApplicationT &p_usbMouseApplication)
      : PeriodicTask(p_name, p_priority, p_periodMs, 256), m_usbMouseApplication(p_usbMouseApplication) {
    }

    void
    setUsbMutex(SemaphoreHandle_t p_usbMutex) {
        m_usbMutex = p_usbMutex;
    }
};

struct UsbMouseMover {
    template<
        unsigned nOffset
    >
    class BackAndForthT {
        int x, y, direction;
    public:
        constexpr BackAndForthT(void) : x(0), y(0), direction(1) {

        }

        void
        move(void) {
            direction *= -1;
            y = x = direction * nOffset;
        };

        std::pair<int, int>
        getPosition(void) const {
            return std::pair(x, y);
        }
    };

    template<
        unsigned nRadius,
        unsigned nSpeed,
        int nDirection = 1
    >
    class CircleT {
        int w;

    public:
        constexpr CircleT(void) : w(0) {

        }

        void
        move(void) {
            static_assert((nDirection == 1) || (nDirection == -1));

            if (nDirection > 0) {
                w += nSpeed;
            } else {
                w -= nSpeed;
            }

            w %= 360;
        };

        std::pair<int, int>
        getPosition(void) const {
            static constexpr float pi = 3.14159265359f;
            float rad = w * (pi / 180.0f);

            int x = cosf(rad) * nRadius;
            int y = sinf(rad) * nRadius;

            return std::pair(x, y);
        }
    }; 
};

} /* namespace tasks */

#endif /* __USB_MOUSE_HPP_E09747F0_2BC0_4B9D_8F6B_3A162AFF62E1 */