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

    m_endptBufferDescr.m_rxAddr     = m_usbDevice.mapHostToPeripheral(reinterpret_cast<uintptr_t>(p_buffer));
    m_endptBufferDescr.m_rxCount    = ((blockSz << USB_COUNT0_RX_BLSIZE_Pos) & USB_COUNT0_RX_BLSIZE_Msk)
                                    | ((numBlocks << USB_COUNT0_RX_NUM_BLOCK_Pos) & USB_COUNT0_RX_NUM_BLOCK_Msk);
}

const
OutEndpoint::irq_handler_t OutEndpoint::m_irq_handler[] = {
    { Interrupt_e::e_CorrectTransferRx, &OutEndpoint::handleCorrectTransferRx },
    { Interrupt_e::e_None,              nullptr }
};

void
OutEndpoint::handleCorrectTransferRx(void) const {
    const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;
    const unsigned numHalfWords =(numBytes + (sizeof(uint16_t) / 2)) / sizeof(uint16_t);

    USB_PRINTF("--> OutEndpoint::%s(m_endpointNumber=%d) numBytes=%d\r\n", __func__, m_endpointNumber, numBytes);
    assert(this->m_endpointCallback != nullptr);

    if (numBytes != 0) {
        const OutEndpointCallback::DataBuffer_t &dataBuffer = this->m_endpointCallback->getDataBuffer();

        /* TODO We require the upper layer to fit the entire packet into RAM for now. */
        assert(dataBuffer.m_buffer != nullptr);
        assert(dataBuffer.m_numHalfWords * sizeof(*dataBuffer.m_buffer) >= numBytes);

        for (unsigned idx = 0; idx < numHalfWords; idx++) {
            dataBuffer.m_buffer[idx] = m_buffer[idx].data;
        }

        this->m_endpointCallback->packetReceived(numBytes);
    }

    this->m_endpointCallback->transferComplete();
    this->rxEnable();

    USB_PRINTF("<-- OutEndpoint::%s()\r\n", __func__);
}

void
OutEndpoint::handleIrq(void) const {
    unsigned irqStatus = *(this->m_register);
    uint16_t handledIrq = 0;

    USB_PRINTF("--> OutEndpoint::%s(m_endpointNumber=%d, irqStatus=0x%x)\r\n", __func__, m_endpointNumber, irqStatus);

    assert(((irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);
    assert(this->m_buffer != nullptr);

    for (const OutEndpoint::irq_handler_t *cur = OutEndpoint::m_irq_handler; cur->m_irq != OutEndpoint::Interrupt_t::e_None; cur++) {
        if (irqStatus & static_cast<uint16_t>(cur->m_irq)) {
            handledIrq |= static_cast<uint16_t>(cur->m_irq);
            (this->*(cur->m_fn))(); // Call member function via pointer
            clearInterrupt(cur->m_irq);
        }
    }

    USB_PRINTF("<-- OutEndpoint::%s()\r\n", __func__);
}

const
CtrlOutEndpoint::irq_handler_t CtrlOutEndpoint::m_irq_handler[] = {
    { Interrupt_e::e_SetupComplete,     &CtrlOutEndpoint::handleSetupComplete },
    { Interrupt_e::e_None,              nullptr }
};

void
CtrlOutEndpoint::handleSetupComplete(void) {
    USB_PRINTF("--> CtrlOutEndpoint::%s(m_endpointNumber=%d)\r\n", __func__, m_endpointNumber);

    const unsigned numBytes = (this->m_endptBufferDescr.m_rxCount >> USB_COUNT0_RX_COUNT0_RX_Pos) & USB_COUNT0_RX_COUNT0_RX_Msk;
    const unsigned numHalfWords = numBytes / sizeof(uint16_t);

    assert(numBytes == sizeof(::usb::UsbSetupPacket_t));                        // Expect 8 Bytes
    assert(numHalfWords == sizeof(::usb::UsbSetupPacket_t) / sizeof(uint16_t)); // Expect 4 Half-Words

    for (unsigned idx = 0; idx < numHalfWords; idx++) {
        m_setupPacketBuffer.getUint16(idx) = m_buffer[idx].data;
    }

    this->m_endpointCallout.setupComplete(this->m_setupPacketBuffer);
    this->enableSetupPackets();

    USB_PRINTF("<-- CtrlOutEndpoint::%s()\r\n", __func__);
}

void
CtrlOutEndpoint::handleIrq(void) {
    const unsigned irqStatus = *(this->m_register);
    uint16_t handledIrq = 0;

    USB_PRINTF("--> CtrlOutEndpoint::%s(m_endpointNumber=%d, irqStatus=0x%x)\r\n", __func__, m_endpointNumber, irqStatus);

    assert(((irqStatus >> USB_EP0R_EA_Pos) & USB_EP0R_EA_Msk) == this->m_endpointNumber);
    assert(this->m_buffer != nullptr);

    for (const CtrlOutEndpoint::irq_handler_t *cur = CtrlOutEndpoint::m_irq_handler; cur->m_irq != CtrlOutEndpoint::Interrupt_t::e_None; cur++) {
        if (irqStatus & static_cast<uint16_t>(cur->m_irq)) {
            handledIrq |= static_cast<uint16_t>(cur->m_irq);
            (this->*(cur->m_fn))(); // Call member function via pointer
            clearInterrupt(cur->m_irq);
        }
    }

    OutEndpoint::handleIrq();

    USB_PRINTF("<-- CtrlOutEndpoint::%s()\r\n", __func__);
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
