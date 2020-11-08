/*-
 * $Copyright$
-*/

#ifndef _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942
#define _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942

#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbTypes.hpp>

#include <stddef.h>
#include <cassert>

#include <unistd.h>

#include <algorithm>


/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

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

    if (this->m_endpointNumber == 0) {
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
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d, p_data=%p, p_length=%d)\r\n", __func__, m_endpointNumber, p_data, p_length);

    this->startTx(p_length);
    this->txData(p_data, p_length);
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
InEndpointViaSTM32F4::startTx(size_t p_numBytes) {
    /* FIXME Deadlock potential here. What if the Hardware never clears the EPENA Bit? */
    while (this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA_Msk);

    /* Setup Transfer Buffer (Number of Packets, Number of Bytes) */
    unsigned numBytes = p_numBytes;
    unsigned packetSz = this->getPacketSize();
    unsigned numPackets = this->getNumPackets(p_numBytes, packetSz);

    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) numBytes=%d packetSz=%d numPackets=%d\r\n", __func__, m_endpointNumber, numBytes, packetSz, numPackets);

    /*
     * The PKTCNT field is decremented every time the USB Hardware reads a packet (maximum size
     * or short packet) from the TxFIFO.
     *
     * The XFRSIZ field initially contains the transfer size in bytes. The USB Hardware decrements
     * this field every time a packet from the external memory is written to the TxFIFO.
     */
    this->m_endpoint->DIEPTSIZ = ((numPackets << USB_OTG_DIEPTSIZ_PKTCNT_Pos) & USB_OTG_DIEPTSIZ_PKTCNT_Msk)
            | ((numBytes << USB_OTG_DIEPTSIZ_XFRSIZ_Pos) & USB_OTG_DIEPTSIZ_XFRSIZ_Msk);

    /*
     * Trigger USB Hardware to start transmission.
     *
     * Setting the EPENA Bit seems to put the core in a "fetch mode" where it attempts
     * to read Tx Data from the IN Endpoint's FIFO. While the FIFO does not contain any
     * data, the "Tx FIFO Empty" IRQ is asserted.
     *
     * We therefore seem to have two options here:
     * a) Have the "Tx FIFO Empty" IRQ handler push data into the FIFO. In this case, we
     *    need to enable the FIFO Irq here.
     * b) Push Tx Data into the FIFO from the same thread / call stack that triggered the
     *    Transmission. In that case, we need to leave the FIFO Irq disabled. Otherwise,
     *    we might end up in an IRQ loop on the "Tx FIFO Empty" IRQ.
     *
     * For now, Option b) is implemented.
     *
     *  The upside of b) is that is't easier to implement. The downsize is that the call
     * triggering the Transmission will either need to block until all Tx Data has been
     * pushed into the Tx FIFO or drop data if the Tx FIFO is not large enough.
     */
    this->m_endpoint->DIEPCTL |= (USB_OTG_DIEPCTL_CNAK | USB_OTG_DIEPCTL_EPENA /* | USB_OTG_DIEPCTL_SD0PID_SEVNFRM */);
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
    // { USB_OTG_DIEPINT_TXFE,     &InEndpointViaSTM32F4::handleTxFifoEmpty },
    // { USB_OTG_DIEPINT_ITTXFE,   &InEndpointViaSTM32F4::handleInTokenWhenTxFifoEmpty },
    // { USB_OTG_DIEPINT_INEPNE,   &InEndpointViaSTM32F4::handleNakEffective },
    // { USB_OTG_DIEPINT_TOC,      &InEndpointViaSTM32F4::handleTimeoutCondition },
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
        }
    }

    /*
     * Clear all IRQs, even the ones not pending. Otherwise we risk an IRQ storm when no
     * IRQ handler function is set up.
     */
    this->m_endpoint->DIEPINT |= irq;

    USB_PRINTF("<-- InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPINT=0x%x\r\n", __func__, this->m_endpointNumber, handledIrq);
}

/***************************************************************************//**
 * @brief Copy Raw Data Buffer to Tx FIFO.
 *
 * This method will block if there is not enough space in the Tx FIFO to receive
 * the data.
 ******************************************************************************/
void
InEndpointViaSTM32F4::txData(const uint8_t *p_data, const size_t p_length) {
    uint32_t freeWordsInFifo = this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV_Msk;
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d, p_length=%d) freeWordsInFifo=%d\r\n", __func__, this->m_endpointNumber, p_length, freeWordsInFifo);

    if ((freeWordsInFifo * sizeof(uint32_t)) < p_length) {
        return;
    }

    unsigned offset = 0;

    while (offset < p_length) {
        /*
         * In case offset + 4 would exceed the m_len of m_data, then copy the remaining
         * data into a temporary buffer byte-wise and fill the Tx FIFO from the temporary
         * buffer.
         *
         * This is to avoid reading from invalid addresses when filling the Tx FIFO.
         */
        const uint32_t * const data = reinterpret_cast<const uint32_t *>(p_data + offset);
        *this->m_fifoAddr = *data;

        offset += sizeof(uint32_t);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTxFifoEmpty(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    assert((this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) != 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTransferComplete(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    this->disableFifoIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleNakEffective(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    // this->m_usbDevice.flushTxFifo(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleInTokenWhenTxFifoEmpty(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);
    // assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTimeoutCondition(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleEndpointDisabled(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    assert((this->m_endpoint->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == 0);

    this->disableIrq();
    this->disableFifoIrq();
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

    this->disableFifoIrq();
    this->enableIrq();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::setupEndpointType(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const {
    /* FIXME Endpoint Types != Bulk or Control are not yet supported. */
    assert(((this->m_endpointNumber == 0) && (p_endpointType == UsbDeviceViaSTM32F4::EndpointType_e::e_Control))
      || (p_endpointType == UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk)
      || (p_endpointType == UsbDeviceViaSTM32F4::EndpointType_e::e_Interrupt));

    this->m_endpoint->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPTYP_Msk);
    this->m_endpoint->DIEPCTL |= (p_endpointType << USB_OTG_DIEPCTL_EPTYP_Pos) & USB_OTG_DIEPCTL_EPTYP_Msk;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::disable(void) const {
    this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_NAKSTS;

    this->disableIrq();
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942 */
