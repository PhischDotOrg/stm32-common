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
    const auto bufferEnd = m_buffer.first + m_buffer.second / sizeof(Device::FifoMemory::m_data);
    const unsigned numFifoWords = 1 + ((p_numBytes + 1) / sizeof(Device::FifoMemory::m_data));
    const auto packetEnd = m_buffer.first + numFifoWords;

    m_endpointCallout.handlePacketReceived<sizeof(Device::FifoMemory::m_data), stm32::copy_from_fifo::ReadCurrentPos, true>(bufferBegin, std::min(bufferEnd, packetEnd));
}

void
OutEndpoint::handleIrq(const uint16_t p_irqStatus) const {
    assert(((p_irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);

    const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;

    if (p_irqStatus & static_cast<uint16_t>(Interrupt_e::e_CorrectTransferRx)) {
        handleCorrectTransferRx(numBytes);

        this->setEPnR(USB_EP0R_CTR_RX_Msk, 0);
        assert((m_hwEndpt.EPxR & USB_EP0R_CTR_RX) == 0);
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
    const auto bufferEnd = m_buffer.first + m_buffer.second;
    const auto packetEnd = m_buffer.first + (p_numBytes / sizeof(Device::FifoMemory::m_data));

    m_endpointCallout.handleSetupPacketReceived<sizeof(Device::FifoMemory::m_data), stm32::copy_from_fifo::ReadCurrentPos>(bufferBegin, std::min(bufferEnd, packetEnd));

    /*
     * Note: The "SETUP Complete" Bit in the IRQ Status Register is not cleared while the CTR_RX Bit
     * is still set. So if an OUT Packet is received (ACK'd) while we're still processing the SETUP
     * Packet, i.e. haven't left the IRQ handler for the SETUP Packet, the IRQ for handling the OUT
     * Packet may still have the "SETUP Complete" Bit set. To prevent this, we need to clear the
     * CTR_RX Bit after we have read the data from the Rx FIFO and before processing the SETUP
     * Transaction.
     */
    this->setEPnR(USB_EP0R_CTR_RX_Msk, 0);
    assert((m_hwEndpt.EPxR & USB_EP0R_CTR_RX) == 0);

    m_endpointCallout.notifySetupPacketReceived();
}

void
CtrlOutEndpoint::handleIrq(uint16_t p_irqStatus) const {
    assert(((p_irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);

    if ((p_irqStatus & static_cast<uint16_t>(Interrupt_e::e_SetupComplete))) {
        const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;

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
