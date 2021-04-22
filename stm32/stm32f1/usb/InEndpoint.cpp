/*-
 * $Copyright$
-*/

#include "usb/InEndpoint.hpp"
#include <usb/UsbTypes.hpp>

#include <algorithm/fifocopy.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/
void
InEndpoint::setupTxBuffer(const void * const p_buffer, const size_t p_length) const {
    m_endptBufferDescr.m_txAddr     = m_usbDevice.mapHostToPeripheral(p_buffer);
    m_endptBufferDescr.m_txCount    = p_length;
}

void
InEndpoint::handleIrq(const uint16_t p_irqStatus) const {
    assert(((p_irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);

    if (p_irqStatus & static_cast<uint16_t>(Interrupt_e::e_CorrectTransferTx)) {
        handleCorrectTransferTx();
    }
}

void
InEndpoint::handleCorrectTransferTx(void) const {
    this->setEPnR(USB_EP0R_CTR_TX_Msk, 0);
    assert((m_hwEndpt.EPxR & USB_EP0R_CTR_TX) == 0);

    this->m_endpointCallback->handlePacketTransmitted(m_endptBufferDescr.m_txCount);
}

size_t
InEndpoint::enqueueInPacket(void) const {
    const auto bufferBegin = m_buffer.first;
    const auto bufferEnd = m_buffer.first + m_buffer.second / sizeof(UsbMem::data); 

    assert(this->m_endpointCallback != nullptr);

    size_t txLength = m_endpointCallback->handlePacketRead<sizeof(UsbMem::data)>(bufferBegin, bufferEnd);
    assert(txLength <= m_buffer.second);

    setupTxBuffer(m_buffer.first, txLength);

    this->setTxStatus(EndpointStatus_t::e_Valid);

    return txLength;
}

void
InEndpoint::write(const uint8_t * /* p_data */, const size_t /* p_length */) const {  
    assert(getCtrTx() == 0);

    enqueueInPacket();

    while (getCtrTx() == 0) __NOP();

    setEPnR(USB_EP0R_CTR_TX, 0);

    assert(this->m_endpointCallback != nullptr);
    this->m_endpointCallback->handlePacketTransmitted(m_endptBufferDescr.m_txCount);
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
