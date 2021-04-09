/*-
 * $Copyright$
-*/

#include "usb/Device.hpp"

#include "usb/InEndpoint.hpp"
#include "usb/OutEndpoint.hpp"

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

struct EndpointBufferDescriptor_s Device::m_bufferDescriptorTable[m_maxEndpoints];

void
Device::reset(void) const {
    /* Initialize Buffer Descriptor Table */
    for (unsigned idx = 0; idx < m_maxEndpoints; idx++) {
        m_bufferDescriptorTable[idx].m_txAddr   = 0;
        m_bufferDescriptorTable[idx].m_txCount  = 0;
        m_bufferDescriptorTable[idx].m_rxAddr   = 0;
        m_bufferDescriptorTable[idx].m_rxCount  = 0;
    }

    setAddress(0);

    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        InEndpoint *inEp = m_inEndpoints[idx];
        OutEndpoint *outEp = m_outEndpoints[idx];

        /* TODO Setup Endpoints
         * USB_EPRX_STAT    = USB_EP_RX_NAK (0b10)
         * USB_EP_T_FIELD   = Endpoint Type (Bulk, Ctrl, Iso, Irq)
         * USB_EPADDR_FIELD = Endpoint Address (as in USB Descriptor)
         * USB_EP_KIND      = 0b0
         * USB_EPTX_STAT    = USB_EP_TX_NAK (0b10)
         */
        if (inEp != nullptr) {
            inEp->reset();
        }

        if (outEp != nullptr) {
            outEp->reset();
        }
    }
 
    m_usbPeripheral.setBufferTable(this->m_bufferDescriptorTable);

    assert(m_ctrlOutEndpoint != nullptr);
    if (m_ctrlOutEndpoint != nullptr) {
        m_ctrlOutEndpoint->enableSetupPackets();
    }

#if 0
    USB_PRINTF("Device::%s(): m_bufferDescriptorTable=%p\r\n", __func__, m_bufferDescriptorTable);
    for (unsigned idx = 0; idx < 8; idx++) {
        USB_PRINTF("\tm_bufferDescriptorTable[%i] m_rxAddr=0x%x m_rxCount=0x%x m_txAddr=0x%x m_txCount=0x%x\r\n", idx,
          m_bufferDescriptorTable[idx].m_rxAddr,
          m_bufferDescriptorTable[idx].m_rxCount,
          m_bufferDescriptorTable[idx].m_txAddr,
          m_bufferDescriptorTable[idx].m_txCount
        );
    }
    USB_PRINTF("\r\n");
#endif
}

void
Device::handleEndpointIrq(unsigned p_endpointNo, unsigned p_direction) const {
    USB_PRINTF("--> Device::%s(p_endpointNo=%d, p_direction=%d)\r\n", __func__, p_endpointNo, p_direction);

    assert(p_endpointNo < this->m_maxEndpoints);
    assert(p_endpointNo < this->m_maxInEndpoints);
    assert(p_endpointNo < this->m_maxOutEndpoints);

    const InEndpoint * const inEndp = m_inEndpoints[p_endpointNo];
    const OutEndpoint * const outEndp = m_outEndpoints[p_endpointNo];
    CtrlOutEndpoint * const ctrlOutEndp = m_ctrlOutEndpoint;

    if (inEndp != nullptr) {
        inEndp->handleIrq();
    }

    if (p_direction != 0) {
        if (p_endpointNo == 0) {
            assert(ctrlOutEndp != nullptr);
            ctrlOutEndp->handleIrq();
        } else {
            assert(outEndp != nullptr);
            outEndp->handleIrq();
        }
    }

    USB_PRINTF("<-- Device::%s()\r\n", __func__);
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
