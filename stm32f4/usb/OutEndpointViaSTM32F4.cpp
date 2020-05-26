/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a
#define _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a

#include <usb/OutEndpointViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbCtrlOutEndpoint.hpp>

#include <algorithm>

namespace usb {
    namespace stm32f4 {

/***************************************************************************//**
 * \brief Constructor.
 * 
 * Constructs a new ::usb::stm32f4::OutEndpointViaSTM32F4 with the given endpoint
 * number and registers it as a callback receiver with the provided
 * ::usb::stm32f4::UsbDeviceViaSTM32F4 object.
 * 
 * \param p_usbDevice Object that represents the STM32F4 USB Device Hardware Driver.
 * \param p_endpointNumber Endpoint Number without USB direction encoding.
 *
 * \see ::usb::stm32f4::UsbDeviceViaSTM32F4::registerEndpoint
 ******************************************************************************/
OutEndpointViaSTM32F4::OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber)
  : m_usbDevice(p_usbDevice),
    m_endpointNumber(p_endpointNumber),
    m_endpoint(reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_OUT_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
    m_fifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE)))
{
    this->m_usbDevice.registerEndpoint(this->getEndpointNumber(), *this);
}

/***************************************************************************//**
 * @brief Destructor.
 * 
 * Destructs the object and unregisters it at the STM32F4 USB Device Hardware
 * driver referred to by ::m_usbDevice.
 * 
 * \see ::usb::stm32f4::UsbDeviceViaSTM32F4::unregisterEndpoint
 ******************************************************************************/
OutEndpointViaSTM32F4::~OutEndpointViaSTM32F4() {
    this->m_usbDevice.unregisterEndpoint(this->getEndpointNumber());
}

/***************************************************************************//**
 * \brief Disable the OUT Endpoint.
 * 
 * Disables the OUT Endpoint, causing it to NAK any incoming OUT packets.
 * 
 * This is done by setting the \c SNAK and \c EPDIS bits in the \c DOEPCTL
 * register.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::disable(void) const {
    this->m_endpoint->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK | USB_OTG_DOEPCTL_EPDIS);
}

/***************************************************************************//**
 * @brief Enable the OUT Endpoint to receive packets.
 * 
 * This method enables the OUT Endpoint to receive packets from the USB Bus.
 * Internally, this will clear the USB Hardware's _NAK_ Bit for the Endpoint
 * by setting \c CNAK in the \c DOEPCTL register.
 * 
 * Also, ths \c EPENA Bit will be set which enables receiving Packets into the
 * USB Hardware's Rx FIFO.
 * 
 * From a USB Bus perspective, the Endpoint will accept packets after this method
 * has been called.
 * 
 * Please note that ::usb::stm32f4::OutEndpointViaSTM32F4::setup must have been
 * called (to setup Endpoint Type, Packet Size, etc.) before the endpoint can be
 * enabled safely.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::enable(void) const {
    this->m_usbDevice.enableEndpointIrq(*this);

    this->m_endpoint->DOEPCTL |= (USB_OTG_DOEPCTL_CNAK | USB_OTG_DOEPCTL_EPENA);
}

/***************************************************************************//**
 * @brief Set up the max. number of SETUP Packets the endpoint can handle.
 * 
 * \param p_numPackets Max. Number of SETUP Packets the endpoint can handle in
 *   parallel.
 * 
 * Set up the \c PKTCNT bits in the \c DOEPTSIZ register to \p p_numPackets.
 * 
 * \warning For the current implementation, numbers > 1 have not been tested and
 * will lead to a failed \c assert().
 ******************************************************************************/
void
CtrlOutEndpointViaSTM32F4::enableSetupPackets(const unsigned p_numPackets) const {
    assert(p_numPackets >= 1);

	uint32_t reg = this->m_outEndpoint.m_endpoint->DOEPTSIZ;

	reg &= ~USB_OTG_DOEPTSIZ_PKTCNT_Msk;
	reg |= ((p_numPackets << USB_OTG_DOEPTSIZ_PKTCNT_Pos) && USB_OTG_DOEPTSIZ_PKTCNT_Msk);

	this->m_outEndpoint.m_endpoint->DOEPTSIZ = reg;
}

/***************************************************************************//**
 * \brief Set up the endpoint's max. Packet Size.
 * 
 * \param p_packetSize Max. Packet Size for this Endpoint in Bytes.
 *
 * For Control Endpoints, the max. Packet Size is defined by the USB standard.
 * The hardware only supports 8, 16, 32 or 64 Bytes as a valid packet size. This
 * is guarded by an \c assert() statement.
 * 
 * For all other endpoint types, the max. Packet Size is also defined by the USB
 * standard and depends on the enumerated speed. However, no checking is performed.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::setPacketSize(const unsigned p_packetSize) const {
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

    this->m_endpoint->DOEPCTL &= ~USB_OTG_DOEPCTL_MPSIZ_Msk;
    this->m_endpoint->DOEPCTL |= (packetSz << USB_OTG_DOEPCTL_MPSIZ_Pos) & USB_OTG_DOEPCTL_MPSIZ_Msk;
}

/***************************************************************************//**
 * @brief Callback to read a SETUP Packet from the Rx FIFO.
 * 
 * This callback method is called from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleRxFIFO
 * when a SETUP packet is received for the endpoint.
 * 
 * This callback method will read the data from the Rx FIFO and place it in
 * a RAM buffer. The RAM buffer is provided by the hardware-independend layer
 * (referred to by ::usb::UsbHwOutEndpoint::m_endpointCallback).
 ******************************************************************************/
void
CtrlOutEndpointViaSTM32F4::setupDataReceivedDeviceCallback(const size_t p_numBytes) {
    size_t numWords = (p_numBytes + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);

    USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s(p_numBytes=%d)\r\n", __func__, p_numBytes);

    assert(p_numBytes == sizeof(UsbSetupPacket_t));                     // Expect 8 Bytes
    assert(numWords == sizeof(UsbSetupPacket_t) / sizeof(uint32_t));    // Expect two Words

    for (unsigned idx = 0; idx < numWords; idx++) {
        m_setupPacketBuffer[idx] = *(this->m_outEndpoint.m_fifoAddr);
    }
}

/***************************************************************************//**
 * @brief Callback to decode a received SETUP Packet.
 * 
 * This callback method is called from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleRxFIFO
 * when a SETUP packet has been received and is ready to be decoded.
 * 
 * The OUT endpoint's actual _SETUP Complete_ Interrupt is handled in
 * ::usb::stm32f4::OutEndpointViaSTM32F4::handleSetupDoneIrq.
 ******************************************************************************/
void
CtrlOutEndpointViaSTM32F4::setupCompleteDeviceCallback(void) const {
    USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s()\r\n", __func__);

    assert(this->m_outEndpoint.m_endpointNumber == 0);
}

    
/***************************************************************************//**
 * @brief Callback to read received OUT data from the Hardware's Rx FIFO.
 * 
 * This callback method is called from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleRxFIFO
 * when there is OUT data in the USB Device hardware's Rx FIFO.
 * 
 * This method will forward the data to the device-independent layer in
 * ::usb::UsbOutEndpoint (via ::usb::UsbHwOutEndpoint::m_endpointCallback).
 ******************************************************************************/
void
OutEndpointViaSTM32F4::dataReceivedDeviceCallback(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e & /* p_dataPID */) const {
    size_t numWords = (p_numBytes + (sizeof(uint32_t) - 1)) / sizeof(uint32_t);

    USB_PRINTF("OutEndpointViaSTM32F4::%s(p_numBytes=%d)\r\n", __func__, p_numBytes);

    // assert(p_numBytes <= sizeof(this->m_rxBuffer));
    assert(this->m_endpointCallback != NULL);

    /*
     * From my observation, Zero-length Packets trigger a Transfer Complete IRQ
     * on the Endpoint and thus are handled through that path.
     */
    if (p_numBytes != 0) {
        const OutEndpointViaSTM34F4Callback::DataBuffer_t &dataBuffer = this->m_endpointCallback->getDataBuffer();

        assert(dataBuffer.m_buffer != nullptr);
        /* TODO We require the upper layer to fit the entire packet into RAM for now. */
        assert(dataBuffer.m_numWords * sizeof(uint32_t) >= p_numBytes);

        /* FIXME This requires the upper layers to provide Word-aligned buffers.
         * This means that even if only a single byte is to be transferred from
         * USB to RAM, then the upper layer must reserve four bytes.
         */
        for (unsigned idx = 0; idx < std::min(numWords, dataBuffer.m_numWords); idx++) {
            dataBuffer.m_buffer[idx] = *(this->m_fifoAddr);
        }

        this->m_endpointCallback->packetReceived(p_numBytes);
    }
}

/***************************************************************************//**
 * @brief Callback to handle the OUT Transfer Complete Signal.
 * 
 * This is a callback used from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleRxFIFO
 * where the \c PKTSTS Bit of the \c GRXSTSP Register is handled. This is a result
 * of handling the \c RXFLVL Interrupt (part of \c GINTSTS register).
 *
 * I think this bit is set when all of the OUT Transfer's Rx Data has been read
 * from the FIFO into application memory.
 * 
 * The OUT endpoint's actual Transfer Complete Interrupt is handled in
 * ::usb::stm32f4::OutEndpointViaSTM32F4::handleTransferCompleteIrq.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::transferCompleteDeviceCallback(void) const {
    USB_PRINTF("OutEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);
}

/***************************************************************************//**
 * @brief OUT Endpoint Interrupt Handlers.
 * 
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 * 
 * \see ::usb::stm32f4::OutEndpointViaSTM32F4::handleIrq.
 ******************************************************************************/
const
typename OutEndpointViaSTM32F4::irq_handler_t OutEndpointViaSTM32F4::m_irq_handler[] = {
    { Interrupt_e::e_TransferCompleted,     &OutEndpointViaSTM32F4::handleTransferCompleteIrq },
    { Interrupt_e::e_None,                  nullptr }
};

/***************************************************************************//**
 * @brief OUT Endpoint Interrupt Handlers.
 * 
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 * 
 * \see ::usb::stm32f4::OutEndpointViaSTM32F4::handleIrq.
 ******************************************************************************/
const
typename CtrlOutEndpointViaSTM32F4::irq_handler_t CtrlOutEndpointViaSTM32F4::m_irq_handler[] = {
    { OutEndpointViaSTM32F4::Interrupt_e::e_SetupPhaseDone,         &CtrlOutEndpointViaSTM32F4::handleSetupDoneIrq },
    // { OutEndpointViaSTM32F4::Interrupt_e::e_StatusPhaseReceived,    &CtrlOutEndpointViaSTM32F4::handleStatusPhaseReceivedIrq },
    { OutEndpointViaSTM32F4::Interrupt_e::e_None,                   nullptr }
};

/**************************************************************************//**
 * @brief OUT Endpoint Interrupt Callback.
 * 
 * Interrupt handler for OUT Endpoint Interrupts. Called from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleOutEndpointIrq
 * on the right OutEndpointViaSTM32F4 object.
 * 
 * The OUT Endpoint Interrupts handlers are referred in ::usb::stm32f4::OutEndpointViaSTM32F4::m_irq_handler.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleIrq(void) const {
    const uint32_t  irq = this->m_endpoint->DOEPINT;
    uint32_t        handledIrq = 0;

    USB_PRINTF("--> OutEndpointViaSTM32F4::%s(m_endpointNumber=%i, irq=0x%x)\r\n", __func__, this->m_endpointNumber, irq);

    for (const typename OutEndpointViaSTM32F4::irq_handler_t *cur = OutEndpointViaSTM32F4::m_irq_handler; cur->m_irq != OutEndpointViaSTM32F4::Interrupt_e::e_None; cur++) {
        if (irq & cur->m_irq) {
            handledIrq |= cur->m_irq;

            this->m_endpoint->DOEPINT = cur->m_irq;

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }

    USB_PRINTF("<-- OutEndpointViaSTM32F4::%s(m_endpointNumber=%i, handledIrq=0x%x)\r\n", __func__, this->m_endpointNumber, handledIrq);
}

/**************************************************************************//**
 * @brief OUT Endpoint Interrupt Callback.
 * 
 * Interrupt handler for OUT Endpoint Interrupts. Called from ::usb::stm32f4::UsbDeviceViaSTM32F4::handleOutEndpointIrq
 * on the right OutEndpointViaSTM32F4 object.
 * 
 * The OUT Endpoint Interrupts handlers are referred in ::usb::stm32f4::OutEndpointViaSTM32F4::m_irq_handler.
 ******************************************************************************/
void
CtrlOutEndpointViaSTM32F4::handleIrq(void) const {
    const uint32_t  irq = this->m_outEndpoint.m_endpoint->DOEPINT;
    uint32_t        handledIrq = 0;

    USB_PRINTF("--> CtrlOutEndpointViaSTM32F4::%s(irq=0x%x)\r\n", __func__, irq);

    for (const typename CtrlOutEndpointViaSTM32F4::irq_handler_t *cur = CtrlOutEndpointViaSTM32F4::m_irq_handler; cur->m_irq != OutEndpointViaSTM32F4::Interrupt_e::e_None; cur++) {
        if (irq & cur->m_irq) {
            handledIrq |= cur->m_irq;

            this->m_outEndpoint.m_endpoint->DOEPINT = cur->m_irq;

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }

    this->m_outEndpoint.handleIrq();

    USB_PRINTF("<-- CtrlOutEndpointViaSTM32F4::%s(handledIrq=0x%x)\r\n", __func__, handledIrq);
}

/***************************************************************************//**
 * @brief Handle Endpoint Transfer Complete IRQ.
 * 
 * This is the interrupt handler for the \c XFRC bit in the \c DOEPINT register.
 * It notifies the #m_endpointCallback object of the completed transfer to
 * trigger further processing of the data received at the Device.
 * 
 * Also, the USB hardware clears the enable bit after the OUT transfer is
 * complete. This means the endpoint is NAK'ing any incoming OUT packets until
 * we re-enable it. So this method re-enables the endpoint.
 * 
 * \see ::usb::stm32f4::OutEndpointViaSTM32F4::enable()
 ******************************************************************************/
void
OutEndpointViaSTM32F4::handleTransferCompleteIrq(void) const {
    USB_PRINTF("OutEndpointViaSTM32F4::%s(m_endpointNumber=%d)\r\n", __func__, this->m_endpointNumber);

    assert(this->m_endpointCallback != NULL);
    this->m_endpointCallback->transferComplete();

    /* FIXME Should we evaluate a return code here? */
    this->enable();
}

/***************************************************************************//**
 * @brief Handle the OUT Endpoint SETUP Done Interrupt.
 * 
 * Notify the device-independent layer that the SETUP phase is complete and that
 * a SETUP packet can be decoded.
 *
 * Also, the USB hardware clears the enable bit after the SETUP phase is
 * complete. This means the endpoint is NAK'ing any incoming OUT packets, i.e.
 * a potential data stage, until we re-enable it. So this method also re-enables
 * the endpoint.
 * 
 * \see ::usb::stm32f4::OutEndpointViaSTM32F4::enable()
 ******************************************************************************/
void
CtrlOutEndpointViaSTM32F4::handleSetupDoneIrq(void) const {
    USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s()\r\n", __func__);

    this->m_endpointCallout.setupComplete(this->m_setupPacketBuffer);

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
    this->m_outEndpoint.enable();
}

/***************************************************************************//**
 * @brief Set up Endpoint as Bulk OUT Endpoint.
 * 
 * This method will modify the \c DOEPCTL register to set up the endpoint as a
 * Bulk OUT endpoint.
 * 
 * The max. Packet Size depends on the enumerated speed and the max. data buffer
 * that the device-independent layer can handle.
 * 
 * \see ::usb::stm32f4::UsbDeviceViaSTM32F4::getEnumeratedSpeed on querying the
 *   enumerated USB speed.
 * \see ::usb::UsbOutEndpoint::getDataBuffer on obtaining the upper layer's max.
 *   buffer size.
 ******************************************************************************/
void
OutEndpointViaSTM32F4::setup(const UsbDeviceViaSTM32F4::EndpointType_e p_endpointType) const {
    unsigned maxPacketSz, packetSz;

    this->m_endpoint->DOEPCTL &= ~(USB_OTG_DOEPCTL_EPTYP_Msk);
    this->m_endpoint->DOEPCTL |= ((p_endpointType << USB_OTG_DOEPCTL_EPTYP_Pos) & USB_OTG_DOEPCTL_EPTYP_Msk);

    ::usb::UsbHwDevice::DeviceSpeed_e enumeratedSpeed = this->m_usbDevice.getEnumeratedSpeed();
    switch(enumeratedSpeed) {
    case ::usb::UsbHwDevice::DeviceSpeed_e::e_UsbFullSpeed:
        maxPacketSz = 1023;
        break;
    default:
        /* FIXME USB High Speed not (yet?) supported */
        assert(false);

        maxPacketSz = 0;
        break;
    }

    const OutEndpointViaSTM34F4Callback::DataBuffer_t &dataBuffer = this->m_endpointCallback->getDataBuffer();
    packetSz = std::min<unsigned>(dataBuffer.m_numWords * sizeof(uint32_t), maxPacketSz);

    this->setPacketSize(packetSz);

    this->m_endpoint->DOEPCTL |= USB_OTG_DOEPCTL_USBAEP;
}

/**
 * @brief Status Phase Received IRQ Handler.
 *
 * \warning This is left in for completeness. The Data Sheet is unclear on whether
 * this interrupt is also generated in Device Mode.
 */
void
CtrlOutEndpointViaSTM32F4::handleStatusPhaseReceivedIrq(void) const {
    USB_PRINTF("CtrlOutEndpointViaSTM32F4::STATUS Complete %s()\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_CPP_4c0d9db1_5757_4b12_83e7_69b04d8c655a */
