/*-
 * $Copyright$
-*/

#ifndef _TASKS_LED_MATRIX_DISPLAY_HPP_3dcb329a_694a_48b4_8249_5ca9cb2ed9b4
#define _TASKS_LED_MATRIX_DISPLAY_HPP_3dcb329a_694a_48b4_8249_5ca9cb2ed9b4

#include "tasks/Task.hpp"

#include <gpio/GpioPin.hpp>
#include <uart/UartDevice.hpp>

namespace tasks {

template<typename LedMatrixT, typename UartT = uart::UartDevice, unsigned t_width = 8, unsigned t_height = 8>
class LedMatrixDisplayT : public Task {
private:
    UartT &         m_uart;
    LedMatrixT &    m_ledMatrix;
    const unsigned  m_periodUs;

    virtual void run(void);

public:
    LedMatrixDisplayT(const char * const p_name, UartT &p_uart, LedMatrixT &p_ledMatrix, const unsigned p_priority, const unsigned p_periodUs = 500);
    virtual ~LedMatrixDisplayT();
};

}; /* namespace tasks */

#include "LedMatrixDisplay.cpp"

#endif /* _TASKS_LED_MATRIX_DISPLAY_HPP_3dcb329a_694a_48b4_8249_5ca9cb2ed9b4 */ 
