/*-
 * $Copyright$
-*/

#include "usb/InEndpoint.hpp"
#include <usb/UsbTypes.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/
void
InEndpoint::reset(void) const {
    Endpoint::reset();

    m_endptBufferDescr.m_txAddr = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(&(m_buffer[0].data)));
    m_endptBufferDescr.m_txAddr = 0;
}

void
BulkInEndpoint::enable(void) const {
    setEndpointType(EndpointType_t::e_Bulk);
    setTxStatus(TxStatus_t::e_Nak);
}

void
IrqInEndpoint::enable(void) const {
    setEndpointType(EndpointType_t::e_Interrupt);
    setTxStatus(TxStatus_t::e_Nak);
}

void
InEndpoint::handleIrq(void) const {
    unsigned irqStatus = *(this->m_register);
    uint16_t handledIrq = 0;

    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d, irqStatus=0x%x)\r\n", __func__, m_endpointNumber, irqStatus);

    assert(((irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);
    assert(this->m_buffer != nullptr);

    for (const InEndpoint::irq_handler_t *cur = InEndpoint::m_irq_handler; cur->m_irq != InEndpoint::Interrupt_t::e_None; cur++) {
        if (irqStatus & static_cast<uint16_t>(cur->m_irq)) {
            handledIrq |= static_cast<uint16_t>(cur->m_irq);
            (this->*(cur->m_fn))(); // Call member function via pointer
            clearInterrupt(cur->m_irq);
        }
    }

    USB_PRINTF("<-- InEndpoint::%s()\r\n", __func__);
}

const
InEndpoint::irq_handler_t InEndpoint::m_irq_handler[] = {
    { Interrupt_e::e_CorrectTransferTx, &InEndpoint::handleCorrectTransferTx },
    { Interrupt_e::e_None,              nullptr }
};

void
InEndpoint::handleCorrectTransferTx(void) const {
    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d)\r\n", __func__, m_endpointNumber);

    this->m_endptBufferDescr.m_txCount = 0;

    USB_PRINTF("<-- InEndpoint::%s()\r\n", __func__);
}

void
InEndpoint::write(const uint8_t * const p_data, const size_t p_length) const {
    const size_t packetSz = this->m_bufSz;

    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d) p_data=%p p_length=%d\r\n", __func__, m_endpointNumber, p_data, p_length);

    assert(this->m_endptBufferDescr.m_txCount == 0); /* FIXME Should be checked in sendPacket() */

    if (p_length > 0) {
        for (size_t offs = 0; offs < p_length; offs += packetSz) {
            sendPacket(p_data + offs, std::min(packetSz, p_length - offs));
        }
    } else {
        sendPacket(nullptr, 0);
    }

    USB_PRINTF("<-- InEndpoint::%s(m_endpointNumber=%d)\r\n", __func__, m_endpointNumber);
}

void
InEndpoint::sendPacket(const uint8_t * const p_data, const size_t p_length) const {
    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d) p_data=%p, p_length=%d\r\n", __func__, m_endpointNumber, p_data, p_length);

    assert(p_length <= this->m_bufSz);
    this->m_endptBufferDescr.m_txCount = p_length;

    if (p_length > 0) {
        assert(p_data != nullptr);

        // /* FIXME Potentially, this reads from an invalid address beyond p_data. */
        // for (unsigned idx = 0; idx <= p_length; idx += sizeof(uint16_t)) {
        //     m_buffer[idx / 2].data = ((p_data[idx+0] << 0) & 0xFF)
        //                            | ((p_data[idx+1] << 8) & 0xFF);
        // }

        // Baue je zwei Bytes zu einem uint16_t zusammen und schreibe diesen in den USB-Pufferspeicher.
        for (uint_fast16_t i = 0; i < p_length / 2; ++i) {
            uint_fast16_t a = static_cast<uint8_t> (p_data [i*2]);
            uint_fast16_t b = static_cast<uint8_t> (p_data [i*2+1]);

            this->m_buffer [i].data = static_cast<uint16_t> (a | (b << 8));
        }
        // Falls noch ein Byte übrig geblieben ist, schreibe dies einzeln.
        if (p_length % 2) {
            this->m_buffer [p_length/2].data = static_cast<uint8_t> (p_data [p_length-1]);
        }

        /* FIXME Should be done once during setup of Endpoint */
        m_endptBufferDescr.m_txAddr = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(&(m_buffer[0].data)));
    } else {
        /* FIXME Should not be needed. */
        m_endptBufferDescr.m_txAddr = 0;
    }

    this->txEnable();

#if 0
    while ((*this->m_register & USB_EP0R_CTR_TX) == 0) __NOP();
    this->m_endptBufferDescr.m_txCount = 0;
#endif

    USB_PRINTF("<-- InEndpoint::%s()\r\n", __func__);
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
