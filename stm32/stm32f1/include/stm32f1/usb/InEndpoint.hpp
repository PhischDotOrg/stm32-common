/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150
#define __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150

#include "stm32f1/usb/Device.hpp"
#include "stm32f1/usb/Endpoint.hpp"

#include <usb/UsbInEndpoint.hpp>

#include <stm32f4xx.h>

#include <cassert>
#include <cstddef>

#include "f1usb/src/usbutils.hh"

#include <phisch/log.h>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

/*****************************************************************************/
class InEndpoint : protected Endpoint, public ::usb::UsbHwInEndpoint {
    ::usb::UsbInEndpoint *  m_endpointCallback;

    void handleCorrectTransferTx(void) const;
    void setupTxBuffer(const void * const p_buffer, const size_t p_length) const;

protected:
    virtual void setup(void) const = 0;

    size_t enqueueInPacket(void) const;

public:
    InEndpoint(Device &p_usbDevice, Endpoint::Buffer_t p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : Endpoint(p_usbDevice, p_endpointNumber, p_buffer, p_length), m_endpointCallback(nullptr) {
        this->m_usbDevice.registerInEndpoint(this->m_endpointNumber, *this);
    }

    virtual ~InEndpoint() {
        this->m_usbDevice.unregisterInEndpoint(this->m_endpointNumber);
    };

    void reset(void) const {
        Endpoint::reset();
        setupTxBuffer(m_buffer.first, 0);
        setup();
        setDataToggleTx(DataToggleTx_e::e_Data0);
        assert(m_endpointCallback != nullptr);
        m_endpointCallback->reset();
    }

    void handleIrq(const uint16_t p_register) const;

    void
    registerEndpointCallback(::usb::UsbInEndpoint &p_endpointCallback) override {
        assert(m_endpointCallback == nullptr);
        m_endpointCallback = &p_endpointCallback;
    }

    void
    unregisterEndpointCallback(void) override {
        m_endpointCallback = nullptr;
    }

    void stall(void) const override {
        setTxStatus(EndpointStatus_t::e_Stall);
    };

    void nack(void) const override {
        setTxStatus(EndpointStatus_t::e_Nak);
    };

    size_t enableTxPacket(void) const override {
        return enqueueInPacket();
    }

    void write(const uint8_t * p_data, const size_t p_length) const override;

    void setData(unsigned p_dtog) const override {
        setDataToggleTx(p_dtog ? DataToggleTx_e::e_Data1 : DataToggleTx_e::e_Data0);
    }

    bool
    getData(void) const override {
        return (getDataToggleTx() == DataToggleTx_e::e_Data1);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class CtrlInEndpoint : public InEndpoint {
    void setup(void) const override {
        this->setEndpointType(EndpointType_e::e_Control);
    };

public:
    CtrlInEndpoint(Device &p_usbDevice, Endpoint::Buffer_t p_buffer, const size_t p_length)
      : InEndpoint(p_usbDevice, p_buffer, p_length, /* p_endpointNumber = */ 0) {
        // this->m_usbDevice.registerCtrlEndpoint(*this);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class BulkInEndpoint : public InEndpoint {
    void setup(void) const override {
        this->setEndpointType(EndpointType_e::e_Bulk);
    };

public:
    BulkInEndpoint(Device &p_usbDevice, Endpoint::Buffer_t p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : InEndpoint(p_usbDevice, p_buffer, p_length, p_endpointNumber) {
    }
};
/*****************************************************************************/

/*****************************************************************************/
class IrqInEndpoint : public InEndpoint {
    void setup(void) const override {
        this->setEndpointType(EndpointType_e::e_Interrupt);
    };

public:
    IrqInEndpoint(Device &p_usbDevice, Endpoint::Buffer_t p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : InEndpoint(p_usbDevice, p_buffer, p_length, p_endpointNumber) {
    }

    void enable(void) const;

    void disable(void) const {
        InEndpoint::disable();
    }
};
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_INENDPOINT_HPP_8F430B5E_7267_4DAF_9A4E_15DC21305150 */
