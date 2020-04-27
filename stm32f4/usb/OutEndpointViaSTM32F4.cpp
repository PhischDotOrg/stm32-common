/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a
#define _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a

#include <usb/OutEndpointViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbOutEndpoint.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
OutEndpointViaSTM32F4::OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber)
  : m_usbDevice(p_usbDevice),
    m_endpointNumber(p_endpointNumber),
    m_endpoint(reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_OUT_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
    m_fifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE)))
{
    this->m_usbDevice.registerEndpoint(this->getEndpointNumber(), *this);

    this->reset();
}

/*******************************************************************************
 *
 ******************************************************************************/
OutEndpointViaSTM32F4::~OutEndpointViaSTM32F4() {
    this->m_usbDevice.unregisterEndpoint(this->getEndpointNumber(), *this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::reset(void) const {
    this->disable();

    this->m_endpoint->DOEPCTL = 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::disable(void) const {
    this->setNack(true);
    this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::enable(void) const {
    this->m_endpoint->DOEPCTL = ((0x2 << USB_OTG_DOEPCTL_EPTYP_Pos) & USB_OTG_DOEPCTL_EPTYP_Msk)
        | USB_OTG_DOEPCTL_USBAEP | ((64 << USB_OTG_DOEPCTL_MPSIZ_Pos) & USB_OTG_DOEPCTL_MPSIZ_Msk);
    
    if (!(this->m_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)) {
        uint32_t reg = this->m_endpoint->DOEPTSIZ;

        reg &= ~(USB_OTG_DOEPTSIZ_PKTCNT_Msk | USB_OTG_DOEPTSIZ_XFRSIZ_Msk);
        reg |= ((3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) & USB_OTG_DOEPTSIZ_STUPCNT_Msk)
                | ((1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT_Msk)
                | ((16 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);

        this->m_endpoint->DOEPTSIZ = reg;
    }

    this->setNack(false);

    this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_EPENA;
    this->m_endpoint->DOEPCTL &= ~USB_OTG_DOEPCTL_EPDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::setNack(const bool p_nack) const {
    if (p_nack) {
        this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
    } else {
        this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_CNAK;
    }
}

#if 0
/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::enableSetupPackets(const unsigned p_numPackets) const {
	uint32_t reg = this->m_endpoint->DOEPTSIZ;

	reg &= ~USB_OTG_DOEPTSIZ_PKTCNT_Msk;
	reg |= (p_numPackets & 0x3) << USB_OTG_DOEPTSIZ_PKTCNT_Pos;

	this->m_endpoint->DOEPTSIZ = reg;
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleSetupData(const size_t p_numBytes) {
    size_t numWords = (p_numBytes + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);

    assert(p_numBytes == 8);
    assert(numWords == 2);
    assert(this->m_endpointCallback != NULL);

    uint32_t *setupPacketBuffer;
    size_t setupPacketBufferLength;
    
    setupPacketBuffer = static_cast<uint32_t *>(this->m_endpointCallback->getSetupPacketBuffer(&setupPacketBufferLength));
    assert(setupPacketBufferLength >= p_numBytes);

    for (unsigned idx = 0; idx < numWords; idx++) {
        setupPacketBuffer[idx] = *(this->m_fifoAddr);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleSetupComplete(void) {
    assert(this->m_endpointCallback != NULL);

    /* FIXME There is really no need to get and pass the Setup Packet Buffer from the Callback here. */
    size_t setupPacketBufferLength;
    uint32_t *setupPacketBuffer = static_cast<uint32_t *>(this->m_endpointCallback->getSetupPacketBuffer(&setupPacketBufferLength));
    assert(setupPacketBufferLength >= (2 * sizeof(uint32_t)));

    this->m_endpointCallback->setupComplete(setupPacketBuffer, setupPacketBufferLength);
}

    
/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleRxData(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e & /* p_dataPID */) {
    size_t numWords = (p_numBytes + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);

    // assert(p_numBytes <= sizeof(this->m_rxBuffer));
    assert(this->m_endpointCallback != NULL);

    for (unsigned idx = 0; idx < numWords; idx++) {
        this->m_endpointCallback->rxData(*(this->m_fifoAddr));
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleOutTransferComplete(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
const
typename OutEndpointViaSTM32F4::irq_handler_t OutEndpointViaSTM32F4::m_irq_handler[] = {
    { USB_OTG_DOEPINT_STUP, &OutEndpointViaSTM32F4::handleSetupDoneIrq },
    { USB_OTG_DOEPINT_XFRC, &OutEndpointViaSTM32F4::handleTransferComplete },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleIrq(void) {
    const uint32_t  irq = this->m_endpoint->DOEPINT;
    uint32_t        handledIrq = 0;

    for (const typename OutEndpointViaSTM32F4::irq_handler_t *cur = OutEndpointViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        if (irq & cur->m_irq) {
            handledIrq |= cur->m_irq;

            this->m_endpoint->DOEPINT = cur->m_irq;

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleTransferComplete(void) {
    assert(this->m_endpointCallback != NULL);

    this->m_endpointCallback->transferComplete();

    this->enable();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleSetupDoneIrq(void) {
    /*
     * The USB Core clears the Enable Bit in the OTG_FS_DIEPCTL after the Setup
     * Phase is done. We need to re-enable the endpoint (which means also that
     * we need to clear the NAK bit) so the Status Stage of the USB transfer
     * can complete successfully.
     * 
     * The way I understand it, the status stage is: The host sends a zero length
     * packet to the device which the device must ACK. The ACK is generated by
     * the USB Core when a packet is received, the target endpoint is enabled and
     * the NACK bit is not set.
     */
    this->enable();
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a */
