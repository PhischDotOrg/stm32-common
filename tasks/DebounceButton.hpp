/*-
 * $Copyright$
-*/

#ifndef _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5
#define _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5

#include <tasks/PeriodicTask.hpp>
#include <cstdint>

namespace tasks {

template<typename PinT>
class DebounceButtonT : public PeriodicTask {
private:
    const PinT &            m_pin;
    typename PinT::mode_t   m_pinState;

    virtual void executePeriod(void) {
        m_pin.get(m_pinState);
    }

public:
    constexpr DebounceButtonT(const char * const p_name, PinT &p_pin, const unsigned p_priority, const unsigned p_periodMs)
       : PeriodicTask(p_name, p_priority, p_periodMs), m_pin(p_pin), m_pinState(PinT::mode_t::HiZ) {

    };

    virtual ~DebounceButtonT() {

    };
};

}; /* namespace tasks */

#endif /* _DEBOUNCE_BUTTON_HPP_414FDFD7_F850_4E3D_8E6D_510950E1A1A5 */
