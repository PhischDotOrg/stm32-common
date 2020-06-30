/*-
 * $Copyright$
-*/

#ifndef _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2
#define _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2

#include <tasks/PeriodicTask.hpp>

#include <gpio/GpioPin.hpp>
#include <uart/UartDevice.hpp>

namespace tasks {

template<typename UartT = uart::UartDevice, typename PinT = gpio::Pin>
class HeartbeatT : public PeriodicTask {
private:
    const UartT &   m_uart;
    const PinT  &   m_led;
    bool            m_status;

    void executePeriod(void) override {
        this->m_led.set(m_status ? PinT::On : PinT::Off);

        m_status = !m_status;
    }

public:
    HeartbeatT(const char * const p_name, UartT &p_uart, PinT &p_led, const unsigned p_priority, const unsigned p_periodMs = 1000)
      : PeriodicTask(p_name, p_priority, p_periodMs), m_uart(p_uart), m_led(p_led) {

    };

    virtual ~HeartbeatT() override {

    };
};

typedef HeartbeatT<> Heartbeat;

}; /* namespace tasks */

#endif /* _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2 */
