/*-
 * $Copyright$
-*/

#include "usb/OutEndpoint.hpp"
#include <usb/UsbTypes.hpp>

#include <cassert>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

void
OutEndpoint::setupRxBuffer(const void * const p_buffer, const size_t p_length) const {
    assert( ((p_length <= 62) && !(p_length % 2))
            || ((p_length <= 521) && !(p_length % 32)) );

    const unsigned blockSz      = (p_length <= 62) ? 0 : 1;
    const unsigned numBlocks    = blockSz > 0 ? (p_length / 32) - 1 : p_length / 2;

    m_endptBufferDescr.m_rxAddr     = m_usbDevice.mapHostToPeripheral(p_buffer);
    m_endptBufferDescr.m_rxCount    = ((blockSz << USB_COUNT0_RX_BLSIZE_Pos) & USB_COUNT0_RX_BLSIZE_Msk)
                                    | ((numBlocks << USB_COUNT0_RX_NUM_BLOCK_Pos) & USB_COUNT0_RX_NUM_BLOCK_Msk);
}

void
OutEndpoint::handleCorrectTransferRx(const size_t p_numBytes) const {
    /*
     * Pass the received Packet to the upper layers.
     *
     * m_buffer --> Points to USB FIFO Memory (UsbMem)
     *   .first     --> Start Address (UsbMem *); Element Size, i.e. sizeof(UsbMem), is 4 Bytes
     *   .second    --> Length (in Bytes)
     */
    const auto bufferBegin = m_buffer.first;
    const auto bufferEnd = m_buffer.first + m_buffer.second / sizeof(UsbMem::data); 
    const auto packetEnd = m_buffer.first + p_numBytes / sizeof(UsbMem::data);

    m_endpointCallout.handlePacketReceived<sizeof(UsbMem::data)>(bufferBegin, std::min(bufferEnd, packetEnd));
}

void
OutEndpoint::handleIrq(const uint16_t p_irqStatus) const {
    assert(((p_irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);

    this->setEPnR(USB_EP0R_CTR_RX_Msk, 0);
    const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;
    assert((m_hwEndpt.EPxR & USB_EP0R_CTR_RX) == 0);

    if (p_irqStatus & static_cast<uint16_t>(Interrupt_e::e_CorrectTransferRx)) {
        if (m_endpointNumber == 0) setEPnR(USB_EP_KIND_Msk, 0); // TODO Really needed?

        handleCorrectTransferRx(numBytes);
    }
}

void
CtrlOutEndpoint::handleSetupComplete(const size_t p_numBytes) const {
    assert(p_numBytes == sizeof(::usb::UsbSetupPacket_t));

    /*
     * Pass the received Packet to the upper layers.
     *
     * m_buffer --> Points to USB FIFO Memory (UsbMem)
     *   .first     --> Start Address (UsbMem *); Element Size, i.e. sizeof(UsbMem), is 4 Bytes
     *   .second    --> Length (in Bytes)
     */
    const auto bufferBegin = m_buffer.first;
    const auto bufferEnd = m_buffer.first + m_buffer.second / sizeof(UsbMem::data); 
    const auto packetEnd = m_buffer.first + p_numBytes / sizeof(UsbMem::data);

    m_endpointCallout.handleSetupPacketReceived<sizeof(UsbMem::data)>(bufferBegin, std::min(bufferEnd, packetEnd));
}

void
CtrlOutEndpoint::handleIrq(uint16_t p_irqStatus) {
    assert(((p_irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);

    if ((p_irqStatus & static_cast<uint16_t>(Interrupt_e::e_SetupComplete))) {
        this->setEPnR(USB_EP0R_CTR_RX_Msk, 0);
        const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;
        assert((m_hwEndpt.EPxR & USB_EP0R_CTR_RX) == 0);

        handleSetupComplete(numBytes);
    } else {
        OutEndpoint::handleIrq(p_irqStatus);
    }
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
