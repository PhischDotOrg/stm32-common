/*-
 * $Copyright$
-*/

#ifndef _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942
#define _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942

#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbTypes.hpp>

#include <phisch/log.h>

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
InEndpointViaSTM32F4::ack(size_t p_numBytes) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) p_numBytes=%d\r\n", __func__, m_endpointNumber, p_numBytes);

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
     * 
     */
    /* TODO Is this correct for transfers > 1 Packet? */
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
   this->m_endpoint->DIEPCTL |= (/* USB_OTG_DIEPCTL_CNAK | */ USB_OTG_DIEPCTL_EPENA);

    this->enableFifoIrq();
}

/***************************************************************************//**
 * \brief IN Endpoint specific IRQ Handler.
 *
 * IRQ Handler called from ::usb::stm32f4::UsbDeviceViaSTM32F4.
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleIrq(void) const {
    const uint32_t  irq = this->m_endpoint->DIEPINT;
    uint32_t        handledIrq = 0;

    USB_PRINTF("--> InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPINT=0x%x\r\n", __func__, this->m_endpointNumber, irq);

    for (auto cur : InEndpointViaSTM32F4::m_irq_handler) {
        if (irq & static_cast<uint32_t>(cur.m_irq)) {
            (this->*(cur.m_fn))(); // Call member function via pointer
            handledIrq |= static_cast<uint32_t>(cur.m_irq);
        }
    }

    /*
     * Interrupt Flags are cleared by writing 0b1. The Tx FIFO Empty IRQ cannot
     * be cleared by the application. Writing to that bit will have strange effects,
     * e.g. that the Transfer Complete IRQ is not triggering.
     */
    this->m_endpoint->DIEPINT = (handledIrq & ~USB_OTG_DIEPINT_TXFE);

    USB_PRINTF("<-- InEndpointViaSTM32F4::%s(m_endpointNumber=%d) DIEPINT=0x%x\r\n", __func__, this->m_endpointNumber, handledIrq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTxFifoEmpty(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    fillFifo();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::handleTransferComplete(void) const {
    USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    this->disableFifoIrq();

    this->m_endpointCallback->handlePacketTransmitted();
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

    fillFifo();
}

void
InEndpointViaSTM32F4::fillFifo(void) const {
    const uint16_t wordsAvailableInFifo = this->m_endpoint->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV;

    static unsigned cnt = 0;

    if (wordsAvailableInFifo > 0) {
        const auto bufferBegin = this->m_fifoAddr;
        const auto bufferEnd = this->m_fifoAddr + wordsAvailableInFifo;

        assert(this->m_endpointCallback != nullptr);
        size_t txLength = m_endpointCallback->handlePacketRead<sizeof(*this->m_fifoAddr), stm32::copy_to_fifo::PushToBegin>(bufferBegin, bufferEnd);
        assert(txLength <= (this->m_fifoSzInWords * wordsAvailableInFifo));
        assert(txLength <= (this->m_fifoSzInWords * sizeof(*this->m_fifoAddr)));
        (void) txLength;

        USB_PRINTF("InEndpointViaSTM32F4::%s(m_endpointNumber=%d) txLength=%d\r\n", __func__, this->m_endpointNumber, txLength);

        cnt = txLength ? 0 : cnt + 1;

        assert(cnt < 10);

        this->m_endpoint->DIEPCTL = (USB_OTG_DIEPCTL_CNAK /* | USB_OTG_DIEPCTL_EPENA */);
    }
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
InEndpointViaSTM32F4::initialize(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const {
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
    this->m_usbDevice.setupTxFifo(*this);
    this->setupTxFifoNumber(this->m_endpointNumber);
    this->setupEndpointType(p_endpointType);

    this->nack();

    this->disableFifoIrq();

    for (auto irq : m_irq_handler) {
        this->m_usbDevice.enableEndpointCommonIrq(static_cast<UsbDeviceViaSTM32F4::InEndpointIrq_e>(irq.m_irq));
    }

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

    if (m_endpointNumber != 0) {
        this->m_endpoint->DIEPCTL &= ~(USB_OTG_DIEPCTL_EPTYP_Msk);
        this->m_endpoint->DIEPCTL |= (p_endpointType << USB_OTG_DIEPCTL_EPTYP_Pos) & USB_OTG_DIEPCTL_EPTYP_Msk;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
InEndpointViaSTM32F4::disable(void) const {
    this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_NAKSTS;

    this->disableIrq();
}

void
InEndpointViaSTM32F4::stall() const {
    this->m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_STALL;
}

void
InEndpointViaSTM32F4::nack() const {
    m_endpoint->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _INENDPOINT_CPP_a9fc6fbc_90d1_4645_8197_217249205942 */
