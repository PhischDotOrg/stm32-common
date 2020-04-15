/*-
 * $Copyright$
-*/

#ifndef _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942
#define _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942

#include <usb/InEndpoint.hpp>
#include <usb/UsbDevice.hpp>
#include <usb/UsbTypes.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
InEndpointT<UsbDeviceT>::InEndpointT(UsbDeviceT &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
  : m_usbDevice(p_usbDevice), m_endpointNumber(p_endpointNumber),
    m_endpoint(reinterpret_cast<USB_OTG_INEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_IN_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
    m_txFifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE))),
    m_fifoSzInWords(p_fifoSzInWords)
{
    m_usbDevice.registerEndpoint(this->getEndpointNumber(), *this);

    this->reset();

    this->m_usbDevice.setupTxFifo(*this);
    this->setupTxFifoNumber(p_endpointNumber);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
InEndpointT<UsbDeviceT>::~InEndpointT() {
    this->m_usbDevice.unregisterEndpoint(this->getEndpointNumber(), *this);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::reset() const {
    this->m_endpoint->DIEPCTL = 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::setPacketSize(const unsigned p_packetSize) const {
    this->m_endpoint->DIEPCTL |= (p_packetSize << USB_OTG_DIEPCTL_MPSIZ_Pos) & USB_OTG_DIEPCTL_MPSIZ_Msk;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::writeString(const ::usb::UsbStringDescriptor &p_str, const size_t p_len) {
    size_t len = 0;
    
    while (p_str.m_string[len] != '\0') len++;

    this->m_txBuffer.m_isString     = true;
    this->m_txBuffer.m_data.m_str   = p_str.m_string;
    this->m_txBuffer.m_dataLength   = len;
    this->m_txBuffer.m_txLength     = p_len > (2 + 2 * len) ? (2 + 2 * len) : p_len;

    this->startTx();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::write(const uint8_t * const p_data,
  const size_t p_dataLength, const size_t p_txLength) {
    assert(((p_txLength == 0) && (p_dataLength == 0)) || (p_data != NULL));

    this->m_txBuffer.m_isString     = 0;
    this->m_txBuffer.m_data.m_u8    = p_data;
    this->m_txBuffer.m_dataLength   = p_dataLength;
    this->m_txBuffer.m_txLength     = p_txLength > p_dataLength ? p_dataLength : p_txLength;

    this->startTx();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::startTx(void) {
    assert(this->m_txBuffer.m_inProgress == false || this->m_txBuffer.m_data.m_u8 == 0);

    this->m_txBuffer.m_inProgress   = true;

    unsigned numBytes = this->m_txBuffer.m_txLength;
    unsigned numPackets = 1 + (numBytes >> 6);
    assert(numPackets <= 3);

    this->m_endpoint->DIEPTSIZ = (numPackets << USB_OTG_DIEPTSIZ_PKTCNT_Pos)
            | ((numBytes << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);
    this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA;

    this->m_usbDevice.enableEndpointFifoIrq(*this);

    this->enableIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::setNack(const bool p_nack) const {
    if (p_nack) {
        this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_NAKSTS;
    } else {
        this->m_endpoint->DIEPCTL &= ~USB_OTG_DIEPCTL_NAKSTS;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::setupTxFifoNumber(const unsigned p_fifoNumber) const {
    this->m_endpoint->DIEPCTL &= ~USB_OTG_DIEPCTL_TXFNUM;
    this->m_endpoint->DIEPCTL |= (p_fifoNumber << 22) & USB_OTG_DIEPCTL_TXFNUM;
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
typename InEndpointT<UsbDeviceT>::irq_handler_t InEndpointT<UsbDeviceT>::m_irq_handler[] = {
    { USB_OTG_DIEPINT_TXFE,     &InEndpointT<UsbDeviceT>::handleTxFifoEmpty },
    { USB_OTG_DIEPINT_INEPNE,   &InEndpointT<UsbDeviceT>::handleNakEffective },
    { USB_OTG_DIEPINT_ITTXFE,   &InEndpointT<UsbDeviceT>::handleInTokenWhenTxFifoEmpty },
    { USB_OTG_DIEPINT_TOC,      &InEndpointT<UsbDeviceT>::handleTimeoutCondition },
    { USB_OTG_DIEPINT_EPDISD,   &InEndpointT<UsbDeviceT>::handleEndpointDisabled },
    { USB_OTG_DIEPINT_XFRC,     &InEndpointT<UsbDeviceT>::handleTransferComplete },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleIrq(void) {
    const uint32_t  irq = this->m_endpoint->DIEPINT;
    uint32_t        handledIrq = 0;

    this->disableIrq();

    for (const typename InEndpointT<UsbDeviceT>::irq_handler_t *cur = InEndpointT<UsbDeviceT>::m_irq_handler; cur->m_irq != 0; cur++) {
        if (irq & cur->m_irq) {
            (this->*(cur->m_fn))(); // Call member function via pointer
            handledIrq |= cur->m_irq;

            this->m_endpoint->DIEPINT = cur->m_irq;
        }
    }

    this->enableIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::txString(void) {
    union buffer_u {
        uint8_t     m_u8[sizeof(uint32_t)];
        uint16_t    m_u16[sizeof(m_u8) / sizeof(uint16_t)];
        uint32_t    m_u32;
    } tmp;

    unsigned freeWordsInTxFifo = this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV;
    assert(freeWordsInTxFifo >= (this->m_txBuffer.m_txLength / 4));

    /*
     * If we're sending a string descriptor, then we need to set the first by to the
     * total length of the string as per USB 2.0 Spec, Section 9.6.7, Table 9-16.
     *
     * We're using a temporary buffer to construct the first Tx FIFO Word in that case.
     */
    if (this->m_txBuffer.m_offs == 0) {
        tmp.m_u32   = 0;

        tmp.m_u8[0] = 2 + (2 * this->m_txBuffer.m_dataLength);
        tmp.m_u8[1] = ::usb::UsbDescriptorTypeId_t::e_String;

        if ((this->m_txBuffer.m_data.m_u8 != NULL) && (this->m_txBuffer.m_txLength > 2) && (this->m_txBuffer.m_dataLength != 0)) {
            /* USB is little-endian, STM32F4/Cortex-M4 is big-endian */
            tmp.m_u8[2]     = this->m_txBuffer.m_data.m_u8[0];
            tmp.m_u8[3]     = 0;

            this->m_txBuffer.m_offs += 1;
        }

        *this->m_txFifoAddr = tmp.m_u32;
    }

    /*
     * Transmit the rest of the String, if requested.
     */
    for (unsigned bytesTransferred = 2 + 2 * this->m_txBuffer.m_offs;
            (this->m_txBuffer.m_offs < this->m_txBuffer.m_dataLength) && (bytesTransferred < this->m_txBuffer.m_txLength);
            bytesTransferred += sizeof(tmp)
      ) {
        int rem = this->m_txBuffer.m_txLength - (2 + (this->m_txBuffer.m_offs * 2));
        if (rem > 2) {
            rem = sizeof(tmp);
        }

        tmp.m_u32 = 0;
        for (int idx = 0; idx < rem; idx += 2) {
            tmp.m_u8[idx]       = this->m_txBuffer.m_data.m_str[this->m_txBuffer.m_offs++];
            tmp.m_u8[idx + 1]   = 0;
        }

        *this->m_txFifoAddr = tmp.m_u32;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::txData(void) {
    union buffer_u {
        uint8_t     m_u8[sizeof(uint32_t)];
        uint16_t    m_u16[sizeof(m_u8) / sizeof(uint16_t)];
        uint32_t    m_u32;
    } tmp;

    unsigned freeWordsInTxFifo = this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV;

    while(freeWordsInTxFifo-- && (this->m_txBuffer.m_offs < this->m_txBuffer.m_txLength)) {
        /*
         * In case m_offs + 4 would exceed the m_len of m_data, then copy the remaining
         * data into a temporary buffer byte-wise and fill the Tx FIFO from the temporary
         * buffer.
         *
         * This is to avoid reading from invalid addresses when filling the Tx FIFO.
         */
        if ((this->m_txBuffer.m_offs + sizeof(uint32_t)) > this->m_txBuffer.m_txLength) {
            tmp.m_u32 = 0;

            for (unsigned idx = 0; (idx < sizeof(tmp.m_u8)) && ((this->m_txBuffer.m_offs + idx) < this->m_txBuffer.m_txLength); idx++) {
                tmp.m_u8[idx] = this->m_txBuffer.m_data.m_u8[this->m_txBuffer.m_offs + idx];
            }

            *this->m_txFifoAddr = tmp.m_u32;
        } else {
            *this->m_txFifoAddr = this->m_txBuffer.m_data.m_u32[this->m_txBuffer.m_offs / 4];
        }
        this->m_txBuffer.m_offs += sizeof(uint32_t);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleTxFifoEmpty(void) {
    this->m_usbDevice.disableEndpointFifoIrq(*this);

    while ((this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) == 0);

    if (this->m_txBuffer.m_data.m_u8 == NULL) {
        goto out;
    }

    if (this->m_txBuffer.m_isString) {
        this->txString();
        this->m_txBuffer.m_inProgress = false;
    } else {
        this->txData();
    }

out:
    return;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleTransferComplete(void) {
    this->m_txBuffer.m_isString     = 0;
    this->m_txBuffer.m_data.m_u8    = NULL;
    this->m_txBuffer.m_dataLength   = 0;
    this->m_txBuffer.m_txLength     = 0;
    this->m_txBuffer.m_offs         = 0;
    this->m_txBuffer.m_inProgress   = false;
    
    this->m_usbDevice.flushTxFifo(*this);
    this->m_usbDevice.disableEndpointFifoIrq(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleNakEffective(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleInTokenWhenTxFifoEmpty(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleTimeoutCondition(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::handleEndpointDisabled(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::disableIrq(void) const {
    this->m_usbDevice.disableEndpointIrq(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT>
void
InEndpointT<UsbDeviceT>::enableIrq(void) const {
    this->m_usbDevice.enableEndpointIrq(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942 */
