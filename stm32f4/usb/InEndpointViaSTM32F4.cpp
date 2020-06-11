/*-
 * $Copyright$
-*/

#ifndef _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942
#define _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942

#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbTypes.hpp>

#include <unistd.h>

#include <algorithm>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
InEndpointViaSTM32F4::InEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
  : m_usbDevice(p_usbDevice),
    m_endpointNumber(p_endpointNumber),
    m_fifoSzInWords(p_fifoSzInWords),
    m_endpoint(reinterpret_cast<USB_OTG_INEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_IN_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
    m_fifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE)))
{
    this->m_usbDevice.registerInEndpoint(this->getEndpointNumber(), *this);

    this->reset();

    this->m_usbDevice.setupTxFifo(*this);
    this->setupTxFifoNumber(p_endpointNumber);
}

/*******************************************************************************
 *
 ******************************************************************************/
InEndpointViaSTM32F4::~InEndpointViaSTM32F4() {
    this->m_usbDevice.unregisterInEndpoint(this->getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::reset() const {
    this->m_endpoint->DIEPCTL = 0;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::setPacketSize(const unsigned p_packetSize) const {
    uint32_t packetSz = p_packetSize;

    if (!this->m_endpointNumber) {
        switch (p_packetSize) {
        case 64u:
            packetSz = 0;
            break;
        case 32u:
            packetSz = 1;
            break;
        case 16u:
            packetSz = 2;
            break;
        case 8u:
            packetSz = 3;
            break;
        default:
            assert(false);
        }
    }

    this->m_endpoint->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ_Msk;
    this->m_endpoint->DIEPCTL |= (packetSz << USB_OTG_DIEPCTL_MPSIZ_Pos) & USB_OTG_DIEPCTL_MPSIZ_Msk;

    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPCTL=0x%x\r\n", __func__, this->m_endpointNumber, this->m_endpoint->DIEPCTL);
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned
InEndpointViaSTM32F4::getPacketSize(void) const {
    unsigned packetSz = ((this->m_endpoint->DIEPCTL >> USB_OTG_DIEPCTL_MPSIZ_Pos) & USB_OTG_DIEPCTL_MPSIZ_Msk);

    /* Encoding for DIEPCTL0 is different than for the other endpoints */
    if (!this->m_endpointNumber) {
        packetSz = (64 >> packetSz);
    }

    return packetSz;
}

/*******************************************************************************
 *
 ******************************************************************************/
unsigned
InEndpointViaSTM32F4::getNumPackets(const size_t p_txLength, const size_t p_packetSz) const {
    unsigned numPacket;

    if (p_txLength) {
        numPacket = 1 + (p_txLength / p_packetSz);

        if ((p_txLength % p_packetSz) == 0) {
            numPacket += 1;
        }
    } else {
        numPacket = 1;
    }

    return numPacket;
}

/***************************************************************************//**
 * @brief Write USB String Data to IN Endpoint and trigger tansmission to Host.
 * 
 * This method allows a user to transmit USB String data to the Host. The method
 * will block if not enough space is available in the Tx FIFO.
 * 
 * Ultimately, it is used from the USB Default Control Pipe class to transmit
 * String Descriptors.
 * 
 * @see ::usb::UsbControlPipe::getStringDescriptor
 * 
 * @param p_str USB String Descriptor to be transmitted.
 * @param p_len Transmission length. Will be truncated to String Descriptor length.
 ******************************************************************************/
void
InEndpointViaSTM32F4::writeString(const ::usb::UsbStringDescriptor &p_str, const size_t p_len) {
    assert(this->m_txBuffer.m_inProgress == false);
    
    this->m_txBuffer.m_isString     = true;
    this->m_txBuffer.m_data.m_str   = p_str.m_string;
    this->m_txBuffer.m_dataLength   = p_str.m_length;
    this->m_txBuffer.m_txLength     = std::min<size_t>(p_len, (2 + 2 * p_str.m_length));

    USB_PRINTF("InEndpointViaSTM32F4::%s(p_len=%d) m_string='%s' m_length=%d m_txLength=%d\r\n",
      __func__, p_len, p_str.m_string, p_str.m_length, this->m_txBuffer.m_txLength);

    this->startTx();
}

/***************************************************************************//**
 * @brief Write Data to IN Endpoint and trigger transmission to Host.
 * 
 * This method allows a user to trigger data transmission to the Host. The method
 * will block if not enough space is available in the Tx FIFO.
 * 
 * @param p_data Pointer to data buffer that is to be transmitted. Can be \c NULL
 *   in which case an empty packet is transmitted.
 * @param p_length Length of transmission. Can be zero in which case an empty
 *   packet is transmitted. The data buffer pointed to \p p_data must be large
 *   enough and valid. 
 ******************************************************************************/
void
InEndpointViaSTM32F4::write(const uint8_t * const p_data, const size_t p_length) {
    assert(this->m_txBuffer.m_inProgress == false);
    assert((p_length == 0) || (p_data != NULL));

    USB_PRINTF("InEndpointViaSTM32F4::%s(p_data=%p, p_length=%d\r\n", __func__, p_data, p_length);

    this->m_txBuffer.m_isString     = 0;
    this->m_txBuffer.m_data.m_u8    = p_data;
    this->m_txBuffer.m_dataLength   = p_length;
    this->m_txBuffer.m_txLength     = p_length;

    this->startTx();
}

/***************************************************************************//**
 * @brief Start Transmission of Data.
 * 
 * Start Transmission of the requested Data Transmission.
 * 
 * - Sets up the \c DIEPTSIZ register for the requested Data Transmission.
 *   - Number of Packets is the Number of Bytes divided by Packet Size.
 *   - Number of Bytes is the total Transfer Size (in Bytes).
 * - Enable the Endpoint FIFO IRQ in #m_usbDevice.
 * - Start Transmission by setting the \c EPENA bit in the \c DIEPCTL register.
 * - Clear the \c NAK bit.
 * - Fill the Tx FIFO with the Data requested for Transmission via ::usb::stm32f4::InEndpointViaSTM32F4::fillTxFifo
 ******************************************************************************/
void
InEndpointViaSTM32F4::startTx(void) {
    assert(this->m_txBuffer.m_inProgress == false);

    /* FIXME Deadlock potential here. What if the Hardware never clears the EPENA Bit? */
    while (this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA_Msk);

    this->m_txBuffer.m_inProgress   = true;

    /* Setup Transfer Buffer (Number of Packets, Number of Bytes) */
    unsigned numBytes = this->m_txBuffer.m_txLength;
    unsigned packetSz = this->getPacketSize();
    unsigned numPackets = this->getNumPackets(this->m_txBuffer.m_txLength, packetSz);

    USB_PRINTF("InEndpointViaSTM32F4::%s() m_data=%p numBytes=%d packetSz=%d numPackets=%d\r\n", __func__, this->m_txBuffer.m_data, numBytes, packetSz, numPackets);

    /*
     * The PKTCNT field is decremented every time the USB Hardware reads a packet (maximum size
     * or short packet) from the TxFIFO.
     * 
     * The XFRSIZ field initially contains the transfer size in bytes. The USB Hardware decrements
     * this field every time a packet from the external memory is written to the TxFIFO.
     */
    this->m_endpoint->DIEPTSIZ = ((numPackets << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk)
            | ((numBytes << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);

    // /* FIXME If we're filling the Tx FIFO before setting EPENA, then things won't work. */
    // this->fillTxFifo();

    /* Enable IRQs */
    this->m_usbDevice.enableEndpointFifoIrq(*this);

    /* Trigger USB Hardware to start transmission */
    this->m_endpoint->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA /* | USB_OTG_DIEPCTL_SD0PID_SEVNFRM */);

    /* FIXME If we're filling the Tx FIFO before setting EPENA, then things won't work. */
    this->fillTxFifo();

    /* Enable IRQs */
    this->m_usbDevice.enableEndpointFifoIrq(*this);

    /* FIXME Dead-lock potential here!
     * 
     * If we're waiting for the EPENA Bit to clear here, then the Hardware seems to go
     * in some sort of a limbo state where the bit is never cleared. Incidentally, this
     * only happes after a "long" time and not always.
     */
    // while (this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA_Msk);
    // while (this->m_endpoint->DIEPTSIZ & USB_OTG_DIEPTSIZ_PKTCNT_Msk);
}

/***************************************************************************//**
 * @brief Fill Tx FIFO with to-be-transmitted Data.
 * 
 * Copies Data from the Transmission Buffer to the Endpoint's Tx FIFO.
 * String Transfers are handled via ::usb::stm32f4::InEndpointViaSTM32F4::txString.
 * This is because the µC Buffer for String Descriptors encodes \c NULL -terminated
 * ASCII strings, but USB defines a different format.
 * All other transfers are handled via ::usb::stm32f4::InEndpointViaSTM32F4::txData.
 ******************************************************************************/
void
InEndpointViaSTM32F4::fillTxFifo(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s() m_inProgress=%d m_data=%p\r\n", __func__, this->m_txBuffer.m_inProgress, this->m_txBuffer.m_data);

    if (this->m_txBuffer.m_isString) {
        this->txString();
    } else {
        this->txData();
    }
}

/***************************************************************************//**
 * \brief Table of IN Endpoint Interrupt Handlers.
 *
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 * 
 ******************************************************************************/
const
typename InEndpointViaSTM32F4::irq_handler_t InEndpointViaSTM32F4::m_irq_handler[] = {
    { USB_OTG_DIEPINT_XFRC,     &InEndpointViaSTM32F4::handleTransferComplete },
    { USB_OTG_DIEPINT_TXFE,     &InEndpointViaSTM32F4::handleTxFifoEmpty },
    { USB_OTG_DIEPINT_ITTXFE,   &InEndpointViaSTM32F4::handleInTokenWhenTxFifoEmpty },
    // { USB_OTG_DIEPINT_INEPNE,   &InEndpointViaSTM32F4::handleNakEffective },
    { USB_OTG_DIEPINT_TOC,      &InEndpointViaSTM32F4::handleTimeoutCondition },
    { USB_OTG_DIEPINT_EPDISD,   &InEndpointViaSTM32F4::handleEndpointDisabled },
    { 0, NULL }
};

/***************************************************************************//**
 * \brief IN Endpoint specific IRQ Handler.
 * 
 * IRQ Handler called from ::usb::stm32f4::UsbDeviceViaSTM32F4.
 * 
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleIrq(void) {
    const uint32_t  irq = this->m_endpoint->DIEPINT;
    uint32_t        handledIrq = 0;

    USB_PRINTF("--> InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPINT=0x%x\r\n", __func__, this->m_endpointNumber, irq);

    for (const typename InEndpointViaSTM32F4::irq_handler_t *cur = InEndpointViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        if (irq & cur->m_irq) {
            (this->*(cur->m_fn))(); // Call member function via pointer
            handledIrq |= cur->m_irq;

            this->m_endpoint->DIEPINT = cur->m_irq;
        }
    }

    USB_PRINTF("<-- InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPINT=0x%x\r\n", __func__, this->m_endpointNumber, handledIrq);
}

/***************************************************************************//**
 * @brief Copy String Descriptor to Tx FIFO.
 * 
 * The String Desciptors in the µC Memory are encoded as \c NULL terminated ASCII
 * strings to save ROM Space. However, USB defines a different String Format:
 * - First two bytes encodes the String's Length (Little Endian 16 Bit Value).
 * - Remaining Bytes encode 16-Bit Characters, also Little Endian encoded.
 * 
 * A single ASCII character therefore needs to be extended to a two-byte value.
 * In order to safe µC RAM, the encoding is performed when copying String Data
 * to the Tx FIFO.
 * 
 * This method will block if there is not enough space in the Tx FIFO to receive
 * the data.
 ******************************************************************************/
void
InEndpointViaSTM32F4::txString(void) {
    typedef struct UsbStrHeader_s {
        uint8_t m_bLength;
        uint8_t m_bDescriptorType;
        uint8_t m_data[2];
    } UsbStrHeader_t;
    static_assert(sizeof(UsbStrHeader_t) == sizeof(uint32_t));

    union buffer_u {
        UsbStrHeader_t  m_hdr;
        uint8_t         m_u8[4];
        uint16_t        m_u16[2];
        uint32_t        m_u32;
    } buffer;
    static_assert(sizeof(buffer) == sizeof(uint32_t));

    /*
     * If we're sending a string descriptor, then we need to set the first by to the
     * total length of the string as per USB 2.0 Spec, Section 9.6.7, Table 9-16.
     *
     * We're using a temporary buffer to construct the first Tx FIFO Word in that case.
     */
    if (this->m_txBuffer.m_offs == 0) {
        buffer.m_hdr.m_bLength = sizeof(UsbStrHeader_t::m_bLength) + sizeof(UsbStrHeader_t::m_bDescriptorType) + (this->m_txBuffer.m_dataLength) * sizeof(uint16_t);
        buffer.m_hdr.m_bDescriptorType = ::usb::UsbDescriptorTypeId_t::e_String;

        if ((this->m_txBuffer.m_data.m_u8 != NULL) && (this->m_txBuffer.m_dataLength != 0)
          && (this->m_txBuffer.m_txLength > (sizeof(UsbStrHeader_t::m_bLength) + sizeof(UsbStrHeader_t::m_bDescriptorType)))) {
            /* USB is little-endian, STM32F4/Cortex-M4 is big-endian */
            buffer.m_hdr.m_data[0] = this->m_txBuffer.m_data.m_u8[0];
            buffer.m_hdr.m_data[1] = 0;

            this->m_txBuffer.m_offs += 1;
        }

        *this->m_fifoAddr = buffer.m_u32;
    }

    /*
     * Transmit the rest of the String, if requested.
     */
    for (unsigned bytesTransferred = sizeof(UsbStrHeader_t::m_bLength) + sizeof(UsbStrHeader_t::m_bDescriptorType) + (this->m_txBuffer.m_offs * sizeof(uint16_t));
            (this->m_txBuffer.m_offs < this->m_txBuffer.m_dataLength) && (bytesTransferred < this->m_txBuffer.m_txLength);
            bytesTransferred += sizeof(buffer)
      ) {
        buffer.m_u8[0]      = this->m_txBuffer.m_data.m_str[this->m_txBuffer.m_offs++];
        buffer.m_u8[1]      = '\0';

        if (this->m_txBuffer.m_offs < this->m_txBuffer.m_dataLength) {
            buffer.m_u8[2]  = this->m_txBuffer.m_data.m_str[this->m_txBuffer.m_offs++];
            buffer.m_u8[3]  = '\0';
        } else {
            buffer.m_u16[1] = 0;
        }

        while ((this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk) == 0); /* Wait until there is space in Tx FIFO. */

        *this->m_fifoAddr = buffer.m_u32;
    }

    // this->markTxBufferComplete();
}

/***************************************************************************//**
 * @brief Copy Raw Data Buffer to Tx FIFO. 
 * 
 * This method will block if there is not enough space in the Tx FIFO to receive
 * the data.
 ******************************************************************************/
void
InEndpointViaSTM32F4::txData(void) {
    union buffer_u {
        uint8_t     m_u8[sizeof(uint32_t)];
        uint16_t    m_u16[sizeof(m_u8) / sizeof(uint16_t)];
        uint32_t    m_u32;
    } tmp;

    // USB_PRINTF("InEndpointViaSTM32F4::%s(Line %d): m_fifoAddr=%p DIEPTSIZ=0x%x m_txLength=%d freeWordsInTxFifo=%d m_offs=%d\r\n", __func__, __LINE__, this->m_fifoAddr, this->m_endpoint->DIEPTSIZ, this->m_txBuffer.m_txLength, (this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV), this->m_txBuffer.m_offs);

    uint32_t freeWordsInFifo = this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;
    if (this->m_endpointNumber)
        USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%i) freeWordsInFifo=%d\r\n", __func__, this->m_endpointNumber, freeWordsInFifo);

    if ((freeWordsInFifo * 4) < this->m_txBuffer.m_txLength) {
        return;
    }

    while (this->m_txBuffer.m_offs < this->m_txBuffer.m_txLength) {
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

            *this->m_fifoAddr = tmp.m_u32;
        } else {
            *this->m_fifoAddr = this->m_txBuffer.m_data.m_u32[this->m_txBuffer.m_offs / 4];
        }

        this->m_txBuffer.m_offs += sizeof(uint32_t);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTxFifoEmpty(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) m_inProgress=%d m_data=%p\r\n",
      __func__, this->m_endpointNumber, (unsigned) this->m_txBuffer.m_inProgress, this->m_txBuffer.m_data);

    assert((this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) != 0);

    if (this->m_txBuffer.m_inProgress) {
        this->fillTxFifo();
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::markTxBufferComplete(void) {
    if (this->m_txBuffer.m_isString) {
        assert(this->m_txBuffer.m_offs * 2 >= (this->m_txBuffer.m_txLength - 2));
    } else {
        assert(this->m_txBuffer.m_offs >= this->m_txBuffer.m_txLength);
    }

    this->m_txBuffer.m_isString     = 0;
    this->m_txBuffer.m_data.m_u8    = NULL;
    this->m_txBuffer.m_dataLength   = 0;
    this->m_txBuffer.m_txLength     = 0;
    this->m_txBuffer.m_offs         = 0;
    this->m_txBuffer.m_inProgress   = false;

    /* FIXME Why is this needed?
     *
     * If we're compiling w/o USB_DEBUG and we don't do this here, then we seemed to get an IRQ
     * storm (DIEPINT = 0x00002080 or 0x00002090). To me this smells like a race condition between
     * enabling the endpoint, clearing the NAK bit and the transfer complete IRQ.
     */
    this->m_usbDevice.disableEndpointFifoIrq(*this);   
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTransferComplete(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) m_inProgress=%d m_data=%p\r\n",
      __func__, this->m_endpointNumber, this->m_txBuffer.m_inProgress, this->m_txBuffer.m_data);

    if (this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA) {
        this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_EPDIS;
    }

    this->markTxBufferComplete();

    // this->m_usbDevice.flushTxFifo(*this);
    // this->m_usbDevice.disableEndpointFifoIrq(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleNakEffective(void) {
    // USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    this->m_usbDevice.flushTxFifo(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleInTokenWhenTxFifoEmpty(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);
    // assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTimeoutCondition(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleEndpointDisabled(void) {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    assert((this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0);

    this->m_usbDevice.disableEndpointFifoIrq(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::disableIrq(void) const {
    this->m_usbDevice.disableEndpointIrq(*this);
}

/***************************************************************************//**
 * \brief Enable the endpoint's IRQ.
 * 
 * Unmasks the IN Endpoint's IRQ in the STM32F4 USB Device Hardware via
 * ::usb::stm32f4::UsbDeviceViaSTM32F4::enableEndpointIrq.
 ******************************************************************************/
void
InEndpointViaSTM32F4::enableIrq(void) const {
    this->m_usbDevice.enableEndpointIrq(*this);
}

/***************************************************************************//**
 * \brief Enable the IN Endpoint in the STM32F4 Device.
 *
 * This method will enable the IN Endpoint in the STM32F4 Device with the
 * desired attributes. These are mostly operations on the \c DIEPCTL register.
 * 
 * - Packet Size is set up as the minimum of the Endpoint's FIFO Size (#m_fifoSzInWords)
 *   and the enumerated speed's max. Packet Size as per USB specification. So the FIFO
 *   Size can be larger than the max. Packet Size, but not smaller. This is a requirement
 *   from the STM32F4 manual.
 * - The Tx FIFO number is set up to #m_endpointNumber. This means the endpoint number and
 *   the FIFO number will always be the same, even though the USB Hardware theoretically
 *   supports different mappings.
 * - Set up the Endpoint Type according to parameter \p p_endpointType.
 * - Set the \c NAK bit so the USB Host knows there is no data, yet.
 * - Mark the endpoint as active.
 * - Enable the Endpoint's IRQ.
 * 
 * \param p_endpointType Type of Endpoint (Control, Bulk, Interrupt, Isochronous).
 * 
 ******************************************************************************/
void
InEndpointViaSTM32F4::enable(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const {
    uint16_t packetSize;

#if defined(USB_DEBUG)
    const ::usb::UsbHwDevice::DeviceSpeed_e enumeratedSpeed = this->m_usbDevice.getEnumeratedSpeed();
    assert(enumeratedSpeed == ::usb::UsbHwDevice::DeviceSpeed_e::e_UsbFullSpeed);
#endif /* defined(USB_DEBUG) */

    /* FIXME Should Packet Sizes really be a magic numbers here or in the USB Device Class? */
    if (this->m_endpointNumber == 0) {
        /* 64 is the Packet size for a Full Speed Device's Control Endpoint */
        packetSize = 64;
    } else {
        /* 1023 is the max. Packet size for a Full Speed Device */
        packetSize = std::min<uint16_t>(1023u, this->m_fifoSzInWords * sizeof(uint32_t));
    }

    this->setPacketSize(packetSize);

    /* This may be redundant here, should have been done by USB Device after USB Reset. */ 
    this->setupTxFifoNumber(this->m_endpointNumber);

    this->setupEndpointType(p_endpointType);

    /* Make Endpoint active */
    this->m_endpoint->DIEPCTL |= (USB_OTG_DIEPCTL_USBAEP | USB_OTG_DIEPCTL_NAKSTS);

    this->enableIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::setupEndpointType(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const {
    /* FIXME Endpoint Types != Bulk or Control are not yet supported. */
    assert((p_endpointType == UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk)
      || ((this->m_endpointNumber == 0) && (p_endpointType == UsbDeviceViaSTM32F4::EndpointType_e::e_Control)));

    this->m_endpoint->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPTYP_Msk);
    this->m_endpoint->DIEPCTL |= (p_endpointType << USB_OTG_DIEPCTL_EPTYP_Pos) & USB_OTG_DIEPCTL_EPTYP_Msk;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::disable(void) const {
    this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_NAKSTS;

    this->m_usbDevice.disableEndpointFifoIrq(*this);
    this->disableIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942 */
