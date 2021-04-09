/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150
#define __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150

#include "stm32f1/usb/Device.hpp"
#include "stm32f1/usb/Endpoint.hpp"

#include <stm32f4xx.h>

#include <cassert>
#include <cstddef>

#include "f1usb/src/usbutils.hh"

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

/*****************************************************************************/
class InEndpoint : protected Endpoint {
public:
    InEndpoint(Device &p_usbDevice, UsbMem * const p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : Endpoint(p_usbDevice, p_endpointNumber, p_buffer, p_length) {
        this->m_usbDevice.registerInEndpoint(this->m_endpointNumber, *this);
    }

    ~InEndpoint() {
        this->m_usbDevice.unregisterInEndpoint(this->m_endpointNumber);
    };

    void reset(void) const {
        Endpoint::reset();

        m_endptBufferDescr.m_txAddr = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(&(m_buffer[0].data)));
    }

    void handleIrq(void) const;

    void sendPacket(const uint8_t * const p_data, const size_t p_length) const;
    void write(const uint8_t * const p_data, const size_t p_length) const;
};
/*****************************************************************************/

/*****************************************************************************/
class CtrlInEndpoint : private InEndpoint {
public:
    CtrlInEndpoint(Device &p_usbDevice, UsbMem * const p_buffer, const size_t p_length)
      : InEndpoint(p_usbDevice, p_buffer, p_length, /* p_endpointNumber = */ 0) {

    }

    void write(const uint8_t * const p_data, const size_t p_length) const {
        InEndpoint::write(p_data, p_length);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class BulkInEndpoint : private InEndpoint {
public:
    BulkInEndpoint(Device &p_usbDevice, UsbMem * const p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : InEndpoint(p_usbDevice, p_buffer, p_length, p_endpointNumber) {
    }

    void enable(void) const;

    void disable(void) const {
        InEndpoint::disable();
    }

    void write(const uint8_t * const p_data, const size_t p_length) const {
        InEndpoint::write(p_data, p_length);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class IrqInEndpoint {

};
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150 */
