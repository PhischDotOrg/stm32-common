/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_OUTENDPOINT_HPP_9F6817A9_7B79_479D_9A40_7BC560C2BD3C
#define __STM32F1_USB_OUTENDPOINT_HPP_9F6817A9_7B79_479D_9A40_7BC560C2BD3C

#include "stm32f1/usb/Device.hpp"
#include "stm32f1/usb/Endpoint.hpp"
#include <stm32f4xx.h>

#include <usb/UsbOutEndpoint.hpp>

#include <cassert>
#include <cstddef>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

/*****************************************************************************/
class OutEndpoint : protected Endpoint, public ::usb::UsbHwOutEndpoint {
    void handleCorrectTransferRx(const size_t p_numBytes) const;
    void setupRxBuffer(const void * const p_buffer, const size_t p_length) const;

protected:
    ::usb::UsbOutEndpoint & m_endpointCallout;

    virtual void setup(void) const = 0;

    void enableRxPacket(void) const override {
        // if (m_endpointNumber == 0) {
        //     bool statusOut = (p_length == 0);
        //     setEPnR(USB_EP_KIND_Msk, statusOut << USB_EP_KIND_Pos);
        // }
        setRxStatus(EndpointStatus_t::e_Valid);
    }

public:
    OutEndpoint(Device &p_usbDevice, ::usb::UsbOutEndpoint &p_endpointCallout, Endpoint::Buffer_t &p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : Endpoint(p_usbDevice, p_endpointNumber, p_buffer, p_length), m_endpointCallout(p_endpointCallout) {
        this->m_usbDevice.registerOutEndpoint(this->m_endpointNumber, *this);
        m_endpointCallout.registerHwOutEndpoint(*this);
    }

    virtual ~OutEndpoint() {
        m_endptBufferDescr.m_rxAddr     = 0;
        m_endptBufferDescr.m_rxCount    = 0;

        /* TODO Should we disable / unconfigure the Hardware here? */

        m_endpointCallout.unregisterHwOutEndpoint();
        this->m_usbDevice.unregisterOutEndpoint(this->m_endpointNumber);
    }

    void reset(void) const {
        Endpoint::reset();
        setupRxBuffer(m_buffer.first, m_buffer.second);
        // setRxStatus(EndpointStatus_t::e_Disabled);
        setup();
        // setDataToggleRx(DataToggleRx_e::e_Data0);
    }

    void stall(void) const override {
        setRxStatus(EndpointStatus_t::e_Stall);
    }

    void nack(void) const override {
        setRxStatus(EndpointStatus_t::e_Nak);
    }

    void handleIrq(const uint16_t p_register) const;

    void setData(unsigned p_dtog) const override {
        setDataToggleRx(p_dtog ? DataToggleRx_e::e_Data1 : DataToggleRx_e::e_Data0);
    }

    bool
    getData(void) const override {
        return (getDataToggleRx() == DataToggleRx_e::e_Data1);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class CtrlOutEndpoint : public OutEndpoint {
    /**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     */
    ::usb::UsbCtrlOutEndpoint &    m_endpointCallout;

    void handleSetupComplete(const size_t p_numBytes) const;

protected:
    void setup(void) const override {
        setEndpointType(EndpointType_t::e_Control);
        /* FIXME Not the cleanest design... -- Mis-uses setup() to send Reset info to upper layer */
        m_endpointCallout.reset();
    };

public:
    CtrlOutEndpoint(Device &p_usbDevice, ::usb::UsbCtrlOutEndpoint &p_endpointCallout, Endpoint::Buffer_t p_buffer, const size_t p_length)
      : OutEndpoint(p_usbDevice, p_endpointCallout, p_buffer, p_length, 0), m_endpointCallout(p_endpointCallout) {
        this->m_usbDevice.registerCtrlEndpoint(*this);
    };

    virtual ~CtrlOutEndpoint() {
        this->m_usbDevice.unregisterCtrlEndpoint();
    }

    void handleIrq(const uint16_t p_register);
};
/*****************************************************************************/

/*****************************************************************************/
class BulkOutEndpoint : public OutEndpoint {
    void setup(void) const override {
        this->setEndpointType(Endpoint::EndpointType_e::e_Bulk);
    }

public:
    BulkOutEndpoint(Device &p_usbDevice, ::usb::UsbBulkOutEndpoint &p_endpointCallout, Endpoint::Buffer_t p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : OutEndpoint(p_usbDevice, p_endpointCallout, p_buffer, p_length, p_endpointNumber) {

    };
};
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_OUTENDPOINT_HPP_9F6817A9_7B79_479D_9A40_7BC560C2BD3C */
