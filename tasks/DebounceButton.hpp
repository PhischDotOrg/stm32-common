/*-
 * $Copyright$
-*/

#ifndef _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5
#define _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5

#include <tasks/PeriodicTask.hpp>
#include <cstdint>

#include <phisch/log.h>

namespace tasks {

template<typename PinT, bool ActiveLow = false>
class DebounceButtonT : public PeriodicTask {
private:
    const PinT &            m_pin;
    uint32_t                m_register;
    QueueHandle_t           m_buttonHandlerQueue;

    
    virtual int executePeriod(void) {
        typename PinT::mode_t   pinState;
        uint32_t                tmpRegister = m_register;
        bool                    buttonState;
        int                     rc = 0;

        m_pin.get(pinState);

        tmpRegister <<= 1;
        if (ActiveLow ? (pinState == PinT::Off) : (pinState == PinT::On)) {
            tmpRegister |= 1;
        } else { /* PinT:Off as HiZ is never returned. */
            /* prevRegister |= 0; */
        }

        /*
         * This will evaluate to true if either:
         * (A) tmpRegister == 0x00000000 && m_register == 0x80000000
         * (B) tmpRegister == 0xffffffff && m_register == 0x7fffffff
         * 
         * In case (A), we're seeing a rising edge; in case (B) we're seeing a falling edge.
         */
        if ((tmpRegister ^ m_register) == 0x80000000) {
            buttonState = tmpRegister != 0; // false if (A), true if (B) 

            if (xQueueSend(this->m_buttonHandlerQueue, &buttonState, 0) != pdTRUE) {
                PHISCH_LOG("DebounceButtonT::%s(): Failed to post Button State to queue.\r\n", __func__);
                
                rc = -1;
            }
        }

        m_register = tmpRegister;

        return (rc);
    }

public:
    constexpr DebounceButtonT(const char * const p_name, const unsigned p_priority, const unsigned p_periodMs, PinT &p_pin)
       : PeriodicTask(p_name, p_priority, p_periodMs), m_pin(p_pin), m_register(0) {

    };

    virtual ~DebounceButtonT() {

    };

    void setTxQueue(const QueueHandle_t p_buttonHandlerQueue) {
        this->m_buttonHandlerQueue = p_buttonHandlerQueue;
    }
};

}; /* namespace tasks */

#endif /* _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5 */
