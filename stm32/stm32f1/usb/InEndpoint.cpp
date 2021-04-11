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
BulkInEndpoint::enable(void) const {
    setAddress(this->m_endpointNumber);
    setEndpointType(EndpointType_t::e_Bulk);

    m_endptBufferDescr.m_txAddr = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(&(m_buffer->data)));
}

void
IrqInEndpoint::enable(void) const {
    setAddress(this->m_endpointNumber);
    setEndpointType(EndpointType_t::e_Interrupt);
}


void
InEndpoint::handleIrq(void) const {
    unsigned reg = *(this->m_register);
    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d, EPnR=0x%x)\r\n", __func__, m_endpointNumber, reg);

    (void) reg;

    this->m_endptBufferDescr.m_txCount = 0;

    this->clearInterrupt(Interrupt_t::e_CorrectTransferTx);

    USB_PRINTF("<-- InEndpoint::%s()\r\n", __func__);
}

void
InEndpoint::write(const uint8_t * const p_data, const size_t p_length) const {
    const size_t packetSz = this->m_bufSz;

    assert(this->m_endptBufferDescr.m_txCount == 0);

    if (p_length > 0) {
        for (size_t offs = 0; offs < p_length; offs += packetSz) {
            sendPacket(p_data + offs, std::min(packetSz, p_length - offs));
        }
    } else {
        sendPacket(nullptr, 0);
    }

}

void
InEndpoint::sendPacket(const uint8_t * const p_data, const size_t p_length) const {
    USB_PRINTF("--> InEndpoint::%s(m_endpointNumber=%d, p_data=%p, p_length=%d)\r\n", __func__, m_endpointNumber, p_data, p_length);

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

        m_endptBufferDescr.m_txAddr = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(&(m_buffer[0].data)));
    } else {
        m_endptBufferDescr.m_txAddr = 0;
    }

    this->txEnable();

    USB_PRINTF("<-- InEndpoint::%s()\r\n", __func__);
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
