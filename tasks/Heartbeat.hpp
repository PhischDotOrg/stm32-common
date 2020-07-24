/*-
 * $Copyright$
-*/

#ifndef _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2
#define _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2

#include <tasks/PeriodicTask.hpp>

#include <gpio/GpioPin.hpp>
#include <uart/UartDevice.hpp>

namespace tasks {

template<typename PinT = gpio::DigitalOutPin>
class HeartbeatT : public PeriodicTask {
private:
    const PinT  &   m_led;
    bool            m_status;

    int executePeriod(void) override {
        this->m_led.set(m_status);

        m_status = !m_status;

        return (0);
    }

public:
    HeartbeatT(const char * const p_name, PinT &p_led, const unsigned p_priority, const unsigned p_periodMs = 1000)
      : PeriodicTask(p_name, p_priority, p_periodMs), m_led(p_led) {

    };

    virtual ~HeartbeatT() override {

    };
};

typedef HeartbeatT<> Heartbeat;

}; /* namespace tasks */

#endif /* _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2 */
