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

class OutEndpointCallback;

/*****************************************************************************/
class OutEndpoint : protected Endpoint {
    friend class CtrlOutEndpoint;

    typedef void (stm32::f1::usb::OutEndpoint::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /***************************************************************************//**
     * @brief Callback to USB-generic OUT Endpoint implementation.
     ******************************************************************************/
    OutEndpointCallback *   m_endpointCallback;

    void setupRxBuffer(const void * const p_buffer, const size_t p_length) const;

    void handleCorrectTransferRx(void) const;

public:
    OutEndpoint(Device &p_usbDevice, UsbMem * const p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : Endpoint(p_usbDevice, p_endpointNumber, p_buffer, p_length) {
        this->m_usbDevice.registerOutEndpoint(this->m_endpointNumber, *this);
    }

    ~OutEndpoint() {
        m_endptBufferDescr.m_rxAddr     = 0;
        m_endptBufferDescr.m_rxCount    = 0;

        this->m_usbDevice.unregisterOutEndpoint(this->m_endpointNumber);
    }

    /**
     * @brief Register a Callback Object.
     * 
     * The OutEndpointCallback is used to bridge to the hardware-indepentent layer.
     * 
     * The object is notified e.g. in case of a _Transfer Complete_ Interrupt.
     * 
     * \see #handleTransferCompleteIrq
     * 
     * @param p_endpointCallback Reference to a OutEndpointViaSTM34F4Callback object.
     */
    void registerEndpointCallback(OutEndpointCallback &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);
        this->m_endpointCallback = &p_endpointCallback;
    }

    /**
     * @brief Unregisters a Callback Object.
     * 
     * Unregisters a OutEndpointCallback object by setting #m_endpointCallback to
     * \c nullptr .
     */
    void unregisterEndpointCallback(void) {
        assert(this->m_endpointCallback != nullptr);
        this->m_endpointCallback = nullptr;
    }

    void reset(void) const {
        this->setupRxBuffer(&(m_buffer->data), this->m_bufSz);

        Endpoint::reset();
    }

    void handleIrq(void) const;
};
/*****************************************************************************/

/*****************************************************************************/
class OutEndpointCallback : public OutEndpoint {
public:
    typedef struct DataBuffer_s {
        /** @brief Pointer to RAM Buffer that can hold OUT Data. */
        uint16_t *  m_buffer;
        /** @brief Size of Buffer pointed to by #m_buffer in units of Words (4-Bytes). */
        size_t      m_numHalfWords;
    } DataBuffer_t;

protected:
    size_t                  m_transmitLength;
    DataBuffer_t            m_dataBuffer;

    virtual void transferComplete(const size_t p_numBytes) const = 0;

public:
    OutEndpointCallback(Device &p_usbDevice, UsbMem * const p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : OutEndpoint(p_usbDevice, p_buffer, p_length, p_endpointNumber) {

    }

    void packetReceived(const size_t p_numBytes) {
        this->m_transmitLength += p_numBytes;
    };

    void transferComplete(void) {
        this->transferComplete(this->m_transmitLength);

        this->m_transmitLength = 0;
    }

    const DataBuffer_t &getDataBuffer(void) const {
        USB_PRINTF("OutEndpointCallback::%s(p_buffer=%p, p_length=%d)\r\n", __func__, m_dataBuffer.m_buffer, m_dataBuffer.m_numHalfWords * sizeof(*m_dataBuffer.m_buffer));

        return this->m_dataBuffer;
    };

    void setDataBuffer(void * const p_buffer, size_t p_length) {
        m_dataBuffer.m_buffer = static_cast<uint16_t *>(p_buffer);
        m_dataBuffer.m_numHalfWords = p_length / sizeof(*m_dataBuffer.m_buffer);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class CtrlOutEndpoint : public OutEndpointCallback {
    /**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     */
    ::usb::UsbCtrlOutEndpointT<CtrlOutEndpoint> & m_endpointCallout;

    typedef void (stm32::f1::usb::CtrlOutEndpoint::*irq_handler_fn)();

    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /**
     * @brief RAM Buffer for a _SETUP_ Packet.
     * 
     * RAM Buffer into which the _SETUP_ Packet is transferred by #handleSetupComplete.
     * This Buffer is shared with the Hardware-independents layers, i.e. the USB Control
     * Pipe will receive a reference to this buffer to decode the packet.
     * 
     * \see handleSetupComplete
     */
    ::usb::UsbSetupPacket_t m_setupPacketBuffer;

    void handleSetupComplete(void);

    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }

public:
    CtrlOutEndpoint(Device &p_usbDevice, ::usb::UsbCtrlOutEndpointT<CtrlOutEndpoint> &p_endpointCallout, UsbMem *p_buffer, const size_t p_length)
      : OutEndpointCallback(p_usbDevice, p_buffer, p_length, 0), m_endpointCallout(p_endpointCallout) {
        p_endpointCallout.registerHwEndpoint(*this);
        this->registerEndpointCallback(*this);
        this->m_usbDevice.registerCtrlEndpoint(*this);
    };

    void setDataStageBuffer(void * const p_buffer, const size_t p_length) {
        m_dataBuffer.m_buffer = static_cast<uint16_t *>(p_buffer);
        m_dataBuffer.m_numHalfWords = p_length / sizeof(*m_dataBuffer.m_buffer);
    }

    void enableSetupPackets(void) const {
        setEndpointType(EndpointType_t::e_Control);

        this->rxEnable();
    }

    void handleIrq(void);
};
/*****************************************************************************/

/*****************************************************************************/
class BulkOutEndpoint : public OutEndpointCallback {
    /**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     */
    ::usb::UsbBulkOutEndpointT<BulkOutEndpoint> & m_endpointCallout;

    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }

public:
    BulkOutEndpoint(Device &p_usbDevice, ::usb::UsbBulkOutEndpointT<BulkOutEndpoint> &p_endpointCallout, UsbMem * const p_buffer, const size_t p_length, const unsigned p_endpointNumber)
      : OutEndpointCallback(p_usbDevice, p_buffer, p_length, p_endpointNumber), m_endpointCallout(p_endpointCallout) {
        m_endpointCallout.registerHwEndpoint(*this);
        this->registerEndpointCallback(*this);
    };

    ~BulkOutEndpoint() override {
        this->unregisterEndpointCallback();
        m_endpointCallout.unregisterHwEndpoint();
    }

    void setup(void) const {
        this->setAddress(this->m_endpointNumber);
        this->setEndpointType(Endpoint::EndpointType_e::e_Bulk);
    }

    void enable(void) const {
        setup();

        rxEnable();
    }

    void disable(void) const {
        Endpoint::disable();
    }
};
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_OUTENDPOINT_HPP_9F6817A9_7B79_479D_9A40_7BC560C2BD3C */
