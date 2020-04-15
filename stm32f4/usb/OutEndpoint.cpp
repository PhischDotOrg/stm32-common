/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a
#define _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a

#include <usb/OutEndpoint.hpp>
#include <usb/UsbDevice.hpp>

extern "C" void led2_off(void);
extern "C" void led3_off(void);

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
OutEndpointT<UsbDeviceT>::OutEndpointT(UsbDeviceT &p_usbDevice, const unsigned p_endpointNumber)
  : m_usbDevice(p_usbDevice), m_endpointNumber(p_endpointNumber),
    m_endpoint(reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_OUT_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
    m_rxFifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE))),
    m_rxBufferCurPos(0)
{
    m_usbDevice.registerEndpoint(this->getEndpointNumber(), *this);

    this->reset();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
OutEndpointT<UsbDeviceT>::~OutEndpointT() {
    m_usbDevice.unregisterEndpoint(this->getEndpointNumber(), *this);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::reset(void) const {
    this->disable();

    this->m_endpoint->DOEPCTL = 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::disable(void) const {
    this->setNack(true);
    this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_EPDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::enable(void) const {
    if (!(this->m_endpoint->DOEPCTL & USB_OTG_DOEPCTL_EPENA)) {
        uint32_t reg = this->m_endpoint->DOEPTSIZ;

        reg &= ~(USB_OTG_DOEPTSIZ_PKTCNT_Msk | USB_OTG_DOEPTSIZ_XFRSIZ_Msk);
        reg |= ((3 << USB_OTG_DOEPTSIZ_STUPCNT_Pos) & USB_OTG_DOEPTSIZ_STUPCNT_Msk)
                | ((1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos) & USB_OTG_DOEPTSIZ_PKTCNT_Msk)
                | ((16 << USB_OTG_DOEPTSIZ_XFRSIZ_Pos) & USB_OTG_DOEPTSIZ_XFRSIZ_Msk);

        this->m_endpoint->DOEPTSIZ = reg;
    }

    this->setNack(false);

    this->m_endpoint->DOEPCTL &= ~USB_OTG_DOEPCTL_EPDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::setNack(const bool p_nack) const {
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
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::enableSetupPackets(const unsigned p_numPackets) const {
	uint32_t reg = this->m_endpoint->DOEPTSIZ;

	reg &= ~USB_OTG_DOEPTSIZ_PKTCNT_Msk;
	reg |= (p_numPackets & 0x3) << USB_OTG_DOEPTSIZ_PKTCNT_Pos;

	this->m_endpoint->DOEPTSIZ = reg;
}
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::handleRxData(const size_t p_numBytes) {
    size_t numWords = (p_numBytes + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);

    this->m_rxBufferCurPos = 0;

    assert(p_numBytes <= sizeof(this->m_rxBuffer));

    for (unsigned idx = 0; idx < numWords; idx++) {
        this->m_rxBuffer.m_rxBuffer32[this->m_rxBufferCurPos] = *this->m_rxFifoAddr;
        this->m_rxBufferCurPos++;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
template<typename UsbDeviceT>
const
typename OutEndpointT<UsbDeviceT>::irq_handler_t OutEndpointT<UsbDeviceT>::m_irq_handler[] = {
    { USB_OTG_DOEPINT_STUP, &OutEndpointT<UsbDeviceT>::handleSetupDone },
    { USB_OTG_DOEPINT_XFRC, &OutEndpointT<UsbDeviceT>::handleTransferComplete },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::handleIrq(void) {
    const uint32_t  irq = this->m_endpoint->DOEPINT;
    uint32_t        handledIrq = 0;

    for (const typename OutEndpointT<UsbDeviceT>::irq_handler_t *cur = OutEndpointT<UsbDeviceT>::m_irq_handler; cur->m_irq != 0; cur++) {
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
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::handleTransferComplete(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::handleSetupDone(void) {
    const UsbRecipient_t usbRecipient = static_cast<UsbRecipient_t>(this->m_rxBuffer.m_setupPacket.m_bmRequestType & 0x0F);

    switch (usbRecipient) {
    case e_Device:
        this->handleDeviceRequest();
        break;
    case e_Interface:
    case e_Endpoint:
    case e_Other:
    default:
        /* FIXME Not yet implemented */
        assert(false);
        break;
    }

    this->m_rxBufferCurPos = 0;

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
template<typename UsbDeviceT>
void
OutEndpointT<UsbDeviceT>::handleDeviceRequest(void) const {
    switch (this->m_rxBuffer.m_setupPacket.m_bRequest) {
    case e_SetAddress:
        this->m_usbDevice.setAddress(this->m_rxBuffer.m_setupPacket.m_wValue & 0x7F);
        break;
    case e_GetDescriptor:
        this->m_usbDevice.getDescriptor(this->m_rxBuffer.m_setupPacket.m_wValue, this->m_rxBuffer.m_setupPacket.m_wLength);
        break;
    case e_SetConfiguration:
        this->m_usbDevice.setConfiguration(this->m_rxBuffer.m_setupPacket.m_wValue);
        break;
    case e_GetStatus:
        this->m_usbDevice.getStatus(this->m_rxBuffer.m_setupPacket.m_wLength);
        break;
    default:
        assert(false);
        break;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a */
