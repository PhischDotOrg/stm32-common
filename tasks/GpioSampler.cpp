/*-
 * $Copyright$
-*/

#include "tasks/GpioSampler.hpp"

#include "gpio/GpioPin.hpp"
#include "gpio/GpioEngine.hpp"
#include "gpio/GpioAccess.hpp"

#include "uart/Uart.hpp"

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "portmacro.h"
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace tasks {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT, typename PinT>
GpioSamplerT<UartT, PinT>::GpioSamplerT(const char * const p_name, UartT &p_uart, PinT &p_pin,
 const unsigned p_priority, const unsigned p_periodMs /* = 1000 */)
  : Task(p_name, p_priority), m_uart(p_uart), m_pin(p_pin), m_periodMs(p_periodMs) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT, typename PinT>
GpioSamplerT<UartT, PinT>::~GpioSamplerT() {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UartT, typename PinT>
void
GpioSamplerT<UartT, PinT>::run(void) {
    this->m_uart.printf("Task '%s' sampling at %d ms.\r\n", this->m_name, this->m_periodMs);
    typename PinT::mode_t pinState;

    TickType_t period = this->m_periodMs / portTICK_PERIOD_MS;
    do {
        this->m_pin.get(pinState);

        this->m_register = this->m_register - (this->m_register >> 4) + ((pinState == PinT::mode_t::On ? 1u : 0) << 3);

        if (!(this->m_pinState) && (this->m_register >> 7)) {
            this->m_uart.printf("GpioSamplerT::%s(): Button ON\r\n", __func__);
        }

        if ((this->m_pinState) && !(this->m_register >> 7)) {
            this->m_uart.printf("GpioSamplerT::%s(): Button OFF\r\n", __func__);
        }

        this->m_pinState = (this->m_register >> 7) != 0;

        vTaskDelay(period);
    } while (1);
}

} /* namespace tasks */
