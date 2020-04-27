/*-
 * $Copyright$
-*/

#ifndef _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2
#define _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2

#include <tasks/Task.hpp>

#include <gpio/GpioPin.hpp>
#include <uart/UartDevice.hpp>

namespace tasks {

template<typename UartT = uart::UartDevice, typename PinT = gpio::Pin>
class HeartbeatT : public Task {
private:
    UartT &         m_uart;
    PinT  &         m_led;
    const unsigned  m_periodMs;

    virtual void run(void);

public:
    HeartbeatT(const char * const p_name, UartT &p_uart, PinT &p_led, const unsigned p_priority, const unsigned p_periodMs = 1000);
    virtual ~HeartbeatT();
};

typedef HeartbeatT<> Heartbeat;

}; /* namespace tasks */

#include <tasks/Heartbeat.cpp>

#endif /* _HEARTBEAT_HPP_1a319f6f_bef1_4ca8_9748_94f746f0eaf2 */
