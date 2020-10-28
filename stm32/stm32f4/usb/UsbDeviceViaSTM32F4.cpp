/*-
 * $Copyright$
-*/

#ifndef _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00
#define	_USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00

#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbCoreViaSTM32F4.hpp>

#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/OutEndpointViaSTM32F4.hpp>

#include <cassert>

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/***************************************************************************//**
 * @brief Constructor.
 * 
 * Constructs a new UsbDeviceViaSTM32F4 object.
 * 
 * @param p_usbCore Reference to the underlying UsbCoreViaSTM32F4 object.
 * @param p_deviceSpeed Speed at which the Device should operate.
 * 
 * \warning As of now, only USB Full Speed is supported.
 ******************************************************************************/
UsbDeviceViaSTM32F4::UsbDeviceViaSTM32F4(UsbCoreViaSTM32F4 &p_usbCore, const DeviceSpeed_e p_deviceSpeed /* = DeviceSpeed_e::e_UsbFullSpeed */)
  : UsbHwDevice(p_deviceSpeed),
    m_usbCore(p_usbCore),
    m_usbDevice(reinterpret_cast<USB_OTG_DeviceTypeDef *>(p_usbCore.getBaseAddr() + USB_OTG_DEVICE_BASE))
{
    m_usbCore.setupMode(UsbCoreViaSTM32F4::e_UsbDevice);
    m_usbCore.registerDevice(*this);

    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_ModeMismatch);

    this->initialize();
}

/***************************************************************************//**
 * @brief Destructor.
 ******************************************************************************/
UsbDeviceViaSTM32F4::~UsbDeviceViaSTM32F4() {
    m_usbCore.unregisterDevice();
}

/***************************************************************************//**
 * @brief Register an IN Endpoint Handler.
 * 
 * Registers an IN Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 * 
 * The method will use \c assert() to make sure \p p_endpointNumber < #m_maxInEndpoints.
 * 
 * The method will use \c assert() to make sure no Endpoint Handler has been registered
 * with the same Endpoint Number before.
 * 
 * @param p_endpointNumber  Number of the Endpoint. Must be smaller than #m_maxInEndpoints.
 * @param p_endpoint        Reference to the InEndpointViaSTM32F4 object.
 * 
 * \see #m_inEndpoints
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::registerInEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] == NULL);

    this->m_inEndpoints[p_endpointNumber] = &p_endpoint;
}

/***************************************************************************//**
 * @brief Unregister an IN Endpoint Handler.
 * 
 * Unregisters an IN Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 * 
 * The method will use \c assert() to make sure \p p_endpointNumber < #m_maxInEndpoints.
 * 
 * The method will use \c assert() to make sure that an Endpoint Handler has been
 * registered with the same Endpoint Number before.
 * 
 * @param p_endpointNumber  Number of the Endpoint. Must be smaller than #m_maxInEndpoints.
 * 
 * \see #m_inEndpoints
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::unregisterInEndpoint(const unsigned p_endpointNumber) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] != nullptr);

    this->m_inEndpoints[p_endpointNumber] = NULL;
}

/***************************************************************************//**
 * @brief Register an OUT Endpoint Handler.
 * 
 * Registers an OUT Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 * 
 * The method will use \c assert() to make sure \p p_endpointNumber < #m_maxOutEndpoints.
 * 
 * The method will use \c assert() to make sure no Endpoint Handler has been registered
 * with the same Endpoint Number before.
 * 
 * @param p_endpointNumber  Number of the Endpoint. Must be smaller than #m_maxOutEndpoints.
 * @param p_endpoint        Reference to the OutEndpointViaSTM32F4 object.
 * 
 * \see #m_outEndpoints
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::registerOutEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] == NULL);

    this->m_outEndpoints[p_endpointNumber] = &p_endpoint;
}

/***************************************************************************//**
 * @brief Unregister an OUT Endpoint Handler.
 * 
 * Unregisters an OUT Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 * 
 * The method will use \c assert() to make sure \p p_endpointNumber < #m_maxOutEndpoints.
 * 
 * The method will use \c assert() to make sure that an Endpoint Handler has been
 * registered with the same Endpoint Number before.
 * 
 * @param p_endpointNumber  Number of the Endpoint. Must be smaller than #m_maxOutEndpoints.
 * 
 * \see #m_outEndpoints
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::unregisterOutEndpoint(const unsigned p_endpointNumber) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] != nullptr);

    this->m_outEndpoints[p_endpointNumber] = NULL;
}

/***************************************************************************//**
 * @brief Register a Control Endpoint Handler.
 * 
 * Registers a Control Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 *
 * The method will use \c assert() to make sure no Endpoint Handler has been registered
 * with the same Endpoint Number before.
 * 
 * @param p_endpoint        Reference to the CtrlOutEndpointViaSTM32F4 object.
 * 
 * \see #m_ctrlOutEndpoint
 *
 ******************************************************************************/void
UsbDeviceViaSTM32F4::registerCtrlEndpoint(CtrlOutEndpointViaSTM32F4 &p_endpoint) {
    assert(this->m_ctrlOutEndpoint == NULL);

    this->m_ctrlOutEndpoint = &p_endpoint;
}

/***************************************************************************//**
 * @brief Unregister a Control Endpoint Handler.
 * 
 * Unregisters a Control Endpoint Handler with the UsbDeviceViaSTM32F4 object.
 * 
 * The method will use \c assert() to make sure that an Endpoint Handler has been
 * registered with the same Endpoint Number before.
 * 
 * \see #m_ctrlOutEndpoint
 *
 ******************************************************************************/void
UsbDeviceViaSTM32F4::unregisterCtrlEndpoint(void) {
    assert(this->m_ctrlOutEndpoint != nullptr);
    this->m_ctrlOutEndpoint = nullptr;
}

/***************************************************************************//**
 * @brief Set up the USB Device Speed in Hardware.
 * 
 * Sets the \c DSPD Bits in the \c DCFG register to set up the USB Device Speed
 * in Hardware.
 * 
 * @param p_speed Selected USB Speed.
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setupDeviceSpeed(const DeviceSpeed_e p_speed) const {
    this->m_usbDevice->DCFG &= ~USB_OTG_DCFG_DSPD;
    this->m_usbDevice->DCFG |= p_speed & USB_OTG_DCFG_DSPD;
}

/***************************************************************************//**
 * @brief Disable all USB Device Interrupts.
 * 
 * This disables all Interrupts from the USB Device, including endpoint interrupts.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableInterrupts(void) const {
    this->m_usbDevice->DIEPMSK      = 0u;
    this->m_usbDevice->DOEPMSK      = 0u;
    this->m_usbDevice->DAINTMSK     = 0u;
    this->m_usbDevice->DIEPEMPMSK   = 0u;
}

#if 0
/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disconnect(void) const {
    this->m_usbDevice->DCTL |= USB_OTG_DCTL_SDIS;

    USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device disconnected\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::connect(void) const {
    this->m_usbDevice->DCTL &= ~USB_OTG_DCTL_SDIS;

    USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device connected\r\n", __func__);
}
#endif

/***************************************************************************//**
 * @brief Handle IN Endpoint IRQs.
 * 
 * Handles the \c IEPINT in the \c GINTSTS register.
 * 
 * Evaluates the \c IEPINT Bits in the \c DAINT register and calls the Interrupt
 * handler method on the right InEndpointViaSTM32F4 object.
 * 
 * If multiple endpoints triggered an interrupt at the same time, then the
 * individual endpoint handlers will be called from low number to high. In other
 * words: Endpoint #0 is called before Endpoint #1, etc.
 * 
 * \see InEndpointViaSTM32F4::handleIrq 
 * \see UsbDeviceViaSTM32F4::m_irq_handler
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleInEndpointIrq(void) const {
    uint16_t irqs = (this->m_usbDevice->DAINT & USB_OTG_DAINT_IEPINT_Msk) >> USB_OTG_DAINT_IEPINT_Pos;

    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        if (irqs & (1 << idx)) {
            if (this->m_inEndpoints[idx] != NULL) {
                this->m_inEndpoints[idx]->handleIrq();
            }
        }
    }
}

/***************************************************************************//**
 * @brief OUT Endpoint IRQ Handler.
 * 
 * Handles \c OEPINT Bit of the \c GINTSTS register. This handler will use the
 * \c OEPINT bits in the \c DAINT register to determine which endpoint has caused
 * the interrupt. If multiple endpoints triggered an interrupt at the same time,
 * then the individual endpoint handlers will be called from low number to high.
 * In other words: Endpoint #0 is called before Endpoint #1, etc.
 * 
 * \see OutEndpointViaSTM32F4::handleIrq 
 * \see CtrlOutEndpointViaSTM32F4::handleIrq 
 * 
 * \see UsbDeviceViaSTM32F4::m_irq_handler
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleOutEndpointIrq(void) const {
    uint16_t irqs = (this->m_usbDevice->DAINT & USB_OTG_DAINT_OEPINT_Msk) >> USB_OTG_DAINT_OEPINT_Pos;

    /*
     * OUT Endpoint 0 is Control Endpoint and thus needs to be handled by the callback
     * object of type CtrlOutEndpointViaSTM32F4 pointed to by m_ctrlOutEndpoint.
     * 
     * This is b/c OutEndpointViaSTM32F4::handleIrq is not a virtual method.
     */
    if (irqs & (1 << 0)) {
        this->m_ctrlOutEndpoint->handleIrq();
    }

    for (unsigned idx = 1; idx < this->m_maxOutEndpoints; idx++) {
        if (irqs & (1 << idx)) {
            if (this->m_outEndpoints[idx] != NULL) {
                this->m_outEndpoints[idx]->handleIrq();
            }
        }
    }
}

#if 0
/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleConnectorIdStatusChangeIrq(void) const {
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleEarlySuspendIrq(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s()\r\n", __func__);  
}
#endif

/***************************************************************************//**
 * \brief Handle the _USB Enumeration Done_ Interrupt.
 * 
 * Handles the \c ENUMDNEM IRQ signalled in the \c GINTSTS register.
 * 
 * The hardware sets this bit and signals this IRQ when the USB Speed
 * Enumeration is complete.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleEnumerationDone(void) const {
#if defined(USB_DEBUG)
    const ::usb::UsbHwDevice::DeviceSpeed_e enumeratedSpeed = this->getEnumeratedSpeed();

    /* FIXME USB High Speed Mode not yet supported */
    assert(enumeratedSpeed == ::usb::UsbHwDevice::DeviceSpeed_e::e_UsbFullSpeed);
#endif /* defined(USB_DEBUG) */

	/*
	 * Because we're a FullSpeed device, we must set the max. packet size for
	 * control packets to 64 Bytes.
	 */
	assert(this->m_outEndpoints[0] != nullptr);
	this->m_outEndpoints[0]->setPacketSize(64);

	assert(this->m_inEndpoints[0] != nullptr);
	this->m_inEndpoints[0]->setPacketSize(64);

    /* Device is ready, so clear global NAK */
	this->m_usbDevice->DCTL |= USB_OTG_DCTL_CGINAK;

	/* Enable Control Endpoints 0 */
	assert(this->m_inEndpoints[0] != NULL);
	assert(this->m_outEndpoints[0] != NULL);
	this->m_ctrlOutEndpoint->enableSetupPackets(1);

	this->m_usbCore.setUsbTurnAroundTime(5); // Value 5 is taken from ST Example Code

	this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_InEndpoint);
	this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_OutEndpoint);
}

/***************************************************************************//**
 * \brief Handle the _USB Reset_ Interrupt.
 * 
 * Handles the \c USBRST IRQ signalled in the \c GINTSTS register.
 * 
 * The hardware sets this bit and signals this IRQ when the Host has issues a
 * USB Reset on the Bus.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleUsbReset(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device reset begin.\r\n", __func__);

    this->setAddress(0);

    /*
     * Endpoint Initialization upon USB Reset as per STM32F4 Datasheet:
     *
     *  1. Set NAK for all OUT Endpoints
     */
    for (unsigned idx = 0; idx < this->m_maxOutEndpoints; idx++) {
        OutEndpointViaSTM32F4 *endpt = this->m_outEndpoints[idx];
        if (endpt != NULL) {
            endpt->disable();
        }
    }

    /*
     * 2. Unmask the following interrupt bits
     *     - INEP0 = 1 in OTG_FS_DAINTMSK (control 0 IN endpoint)
     *     - OUTEP0 = 1 in OTG_FS_DAINTMSK (control 0 OUT endpoint)
     */

    /* Nothing to do here b/c it's done in the Endpoint's enable() method. */

    /*
     *     - STUP = 1 in DOEPMSK (SETUP phase done)
     *     - XFRC = 1 in DOEPMSK (Transfer completed interrupt)
     */
    this->m_usbDevice->DOEPMSK |= (USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM);

    /*
     *     - XFRC = 1 in DIEPMSK (Transfer completed interrupt)
     *     - TOM = 1 in DIEPMSK (Timeout condition)
     */
    this->m_usbDevice->DIEPMSK |= (USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM);

    /*
     * 3. Set up the Data FIFO RAM for each of the FIFOs
     *     - Program the OTG_FS_GRXFSIZ register
     *     - Program the OTG_FS_TX0FSIZ register
     */
    setupTxFifos();

    /* 4. Program the endpoint-specific registers for control OUT endpoint 0
     *    to receive a SETUP packet
     */
    this->m_outEndpoints[0]->enable();

    USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device reset complete.\r\n", __func__);
}

#if 0
/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleStartOfFrame(void) const {

}
#endif

/***************************************************************************//**
 * @brief Rx FIFO Non-Empty IRQ Handler.
 * 
 * Handles the \c RXFLVL Bit in the USB Device's \c GINTSTS register. The USB
 * hardware triggers this interrupt if there is at least one packet pending to
 * be read from the Rx FIFO.
 *
 * This involves reading the \c GRXSTSP register, thereby pop-ing the front
 * of the Rx FIFO queue.
 * 
 * This is the main entry function for handling OUT packets.
 * 
 * - - -
 *
 * For a Control OUT Transfer, the sequence of events is:
 * 
 * 1. SETUP data packet received (\c PKTSTS == \c b0110 ).
 * 
 *    This is handled in CtrlOutEndpointViaSTM32F4::setupDataReceivedDeviceCallback.
 *    
 * 2. Setup Transaction Complete (\c PKTSTS == \c b0100 ).
 * 
 *    This is handled in CtrlOutEndpointViaSTM32F4::setupCompleteDeviceCallback.
 * 
 *    After the Packet has been read from the Rx FIFO, the USB hardware triggers the
 *    _SETUP Done_ Interrupt in the OUT Endpoint (Bit \c STUP in register \c DOEPINT ).
 * 
 *    \see CtrlOutEndpointViaSTM32F4::handleSetupDoneIrq.
 * 
 * 3. OUT Data Packet received (\c PKTSTS == \c b0010 ).
 * 
 *    The OUT Packet is either a zero-length packet to indicate that the Control request doesn't
 *    carry any host-to-device data in its Data Phase.
 * 
 *    Otherwise, if the Data Phase of the Control request needs to transmit data from USB host to
 *    device, then this is when the data should be read from the hardware's Rx FIFO and put into
 *    application RAM for further processing.
 * 
 *    This is handled in ::usb::stm32f4::OutEndpointViaSTM32F4::dataReceivedDeviceCallback.
 * 
 * 4. OUT Transfer completed (\c PKTSTS == \c b0011 ).
 * 
 *    This is handled in ::usb::stm32f4::OutEndpointViaSTM32F4::transferCompleteDeviceCallback.
 * 
 *    This signals that all OUT data has been read from the hardware's Rx FIFO. After this entry
 *    is pop-ed from the \c GRXSTSP register, the USB hardware triggers the
 *    _Transfer Complete_ Interrupt in the OUT Endpoint (Bit \c XFRC in register \c DOEPINT ).
 * 
 *    \see ::usb::stm32f4::OutEndpointViaSTM32F4::handleTransferCompleteIrq.
 * 
 * - - -
 * 
 * For a Bulk OUT Transfer, the sequence of events is:
 * 
 * 1. OUT Data Packet received (\c PKTSTS == \c b0010 ).
 *
 *    This is handled in ::usb::stm32f4::OutEndpointViaSTM32F4::dataReceivedDeviceCallback.
 * 
 * 2. OUT Transfer completed (\c PKTSTS == \c b0011 ).
 *
 *    This is handled in ::usb::stm32f4::OutEndpointViaSTM32F4::transferCompleteDeviceCallback.
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleRxFIFO(void) const {
	uint32_t rxStatus = this->m_usbCore.getRxStatus();

	uint32_t endpointNum = rxStatus & USB_OTG_GRXSTSP_EPNUM;
	uint32_t numBytes = (rxStatus & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;
    DataPID_t dataPID = static_cast<DataPID_e>((rxStatus & USB_OTG_GRXSTSP_DPID_Msk) >> USB_OTG_GRXSTSP_DPID_Pos);
    unsigned packetStatus = (rxStatus & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos;

	switch (packetStatus) {
	case 0x01: /* Global OUT NACK */
        /*
         * From CPU Reference Manual, sec. 34.17.6 / 35.13.7:
         * 
         * PKTSTS = Global OUT NAK, BCNT = 0x000, EPNUM = Don’t Care (0x0), DPID = Don’t Care (0b00).
         * These data indicate that the global OUT NAK bit has taken effect.
         */
        USB_PRINTF("UsbDeviceViaSTM32F4::%s(): Global OUT NACK -- numBytes=%d, endpointNum = %d, dataPID=%x\r\n", __func__, numBytes, endpointNum, dataPID);

        assert(numBytes == 0);
        assert(endpointNum == 0);
		break;
	case 0x02: /* OUT Data Packet Received */
        /*
         * From CPU Reference Manual, sec. 34.17.6 / 35.13.7:
         *
         * PKTSTS = DataOUT, BCNT = size of the received data OUT packet (0 ≤ BCNT ≤ 1024),
         * EPNUM = EPNUM on which the packet was received, DPID = Actual Data PID.
         */
        assert(numBytes <= 1024);

		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->dataReceivedDeviceCallback(numBytes, dataPID);
		}
		break;
	case 0x03: /* OUT Txfr Completed */
        /*
         * From CPU Reference Manual, sec. 34.17.6 / 35.13.7:
         *
         * PKTSTS = Data OUT Transfer Done, BCNT = 0x0, EPNUM = OUT EP Num on which the data transfer
         * is complete, DPID = Don’t Care (0b00).
         * These data indicate that an OUT data transfer for the specified OUT endpoint has completed.
         * After this entry is popped from the receive FIFO, the core asserts a Transfer Completed 
         * interrupt on the specified OUT endpoint.
         */
        assert(numBytes == 0);

#if 0
		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->transferCompleteDeviceCallback();
		}
#endif
		break;
	case 0x04: /* Setup Transaction Complete */
        /*
         * From CPU Reference Manual, sec. 34.17.6 / 35.13.7:
         * 
         * PKTSTS = Setup Stage Done, BCNT = 0x0, EPNUM = Control EP Num, DPID = Don’t Care (0b00).
         * These data indicate that the Setup stage for the specified endpoint has completed and the
         * Data stage has started. After this entry is popped from the receive FIFO, the core asserts
         * a Setup interrupt on the specified control OUT endpoint.
         */
        assert(endpointNum == 0);
        assert(numBytes == 0);
        // assert(dataPID == 0);

#if 0
        assert(this->m_ctrlOutEndpoint != NULL);
        this->m_ctrlOutEndpoint->setupCompleteDeviceCallback();
#endif
		break;
	case 0x06: /* Setup Data Packet Received */
        /*
         * From CPU Reference Manual, sec. 34.17.6 / 35.13.7:
         * 
         * PKTSTS = SETUP, BCNT = 0x008, EPNUM = Control EP Num, DPID = D0. These data indicate that
         * a SETUP packet for the specified endpoint is now available for reading from the receive FIFO.
         */
        assert(endpointNum == 0);
        assert(numBytes == 8);
        assert(dataPID == 0);

        assert(this->m_ctrlOutEndpoint != NULL);
        this->m_ctrlOutEndpoint->setupDataReceivedDeviceCallback(numBytes);
		break;
    default:
        assert(false);
	}
}

/***************************************************************************//**
 * @brief Determine whether the device is in a suspendable State.
 * 
 * Called by UsbDeviceViaSTM32F4::handleUsbSuspendIrq to determine whether the
 * USB Device can be suspended.
 * 
 * \bug Determining whether a USB Device can be suspended is not yet supported.
 *
 * @returns \c True if device can be suspended, \c False otherwise.
 ******************************************************************************/
bool
UsbDeviceViaSTM32F4::isSuspendableState(void) const {
    /* TODO Actually determine suspendable state here */
    return false;
}

/***************************************************************************//**
 * @brief Disable an IN Endpoint's IRQ.
 *
 * Masks the given IN Endpoint's IRQ in the \c DAINTMSK register.
 * 
 * @param p_endpoint Reference to the InEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/***************************************************************************//**
 * @brief Enable the given IN Endpoints IRQs.
 * 
 * This enables the IN Endpoints IRQs in the \c DAINTMSK register.
 * 
 * @param p_endpoint Reference to the InEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/***************************************************************************//**
 * @brief Disable the given IN Endpoints FIFO IRQ.
 * 
 * Masks the given IN Endpoint's Tx FIFO IRQ in the \c DIEPEMPMSK register.
 * 
 * @param p_endpoint Reference to the InEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK &= ~(1 << p_endpoint.getEndpointNumber());
}

/***************************************************************************//**
 * @brief Enables the given IN Endpoints FIFO IRQ.
 * 
 * Unmasks the given IN Endpoint's Tx FIFO IRQ in the \c DIEPEMPMSK register.
 * 
 * @param p_endpoint Reference to the InEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK |= (1 << p_endpoint.getEndpointNumber());
}

/***************************************************************************//**
 * @brief Disable an OUT Endpoint's IRQ.
 *
 * Masks the given OUT Endpoint's IRQ in the \c DAINTMSK register.
 * 
 * @param p_endpoint Reference to the OutEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/***************************************************************************//**
 * @brief Enable the given OUT Endpoints IRQs.
 * 
 * This enables the OUT Endpoints IRQs in the \c DAINTMSK register.
 * 
 * @param p_endpoint Reference to the OutEndpointViaSTM32F4 object.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/***************************************************************************//**
 * @brief Disable an Endpoint's IRQ.
 *
 * Masks the given Endpoint's IRQ in the \c DAINTMSK register.
 * 
 * @param p_endpointNumber Endpoint Number.
 * @param p_isOutEndpoint \c True if the operation should affect an OUT Endpoint,
 *   \c False if it should affect an IN Endpoint.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const {
    this->m_usbDevice->DAINTMSK &= ~(1 << (p_endpointNumber + (p_isOutEndpoint ? 16 : 0)));
}

/***************************************************************************//**
 * @brief Enable an Endpoint's IRQ.
 *
 * Unmasks the given Endpoint's IRQ in the \c DAINTMSK register.
 * 
 * @param p_endpointNumber Endpoint Number.
 * @param p_isOutEndpoint \c True if the operation should affect an OUT Endpoint,
 *   \c False if it should affect an IN Endpoint.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const {
    this->m_usbDevice->DAINTMSK |= 1 << (p_endpointNumber + (p_isOutEndpoint ? 16 : 0));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setAddress(const uint8_t p_address) const {
    assert(p_address < 128);

    this->m_usbDevice->DCFG &= ~USB_OTG_DCFG_DAD;
    this->m_usbDevice->DCFG |= (p_address << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD_Msk;

    USB_PRINTF("UsbDeviceViaSTM32F4::%s(): USB Device Adress = %x\r\n", __func__, p_address);
}

/***************************************************************************//**
 * @brief Start the USB Device.
 *
 * This method activates and starts operation of the USB Device.
 * - The Device Speed is set up via #setupDeviceSpeed
 * - The USB Core is started via UsbCoreViaSTM32F4::start
 * - The IN Endpoint Tx FIFOs are flushed via InEndpointViaSTM32F4::flushTxFifo
 * - The USB Hardware's Device Mode Interrupts are enabled via UsbCoreViaSTM32F4::enableIrq
 *   - Only those Interrupts which actually have a handler in #m_irq_handler are activated.
 * 
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::start(void) const {
    this->setupDeviceSpeed(this->m_deviceSpeed);
    this->m_usbCore.start();

    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        if (this->m_inEndpoints[idx] != NULL) {
            this->flushTxFifo(*(this->m_inEndpoints[idx]));
        }
    }
    
    for (const UsbDeviceViaSTM32F4::irq_handler_t *cur = UsbDeviceViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        this->m_usbCore.enableInterrupt(cur->m_irq);
    }
}

/***************************************************************************//**
 * @brief Stop the USB Device.
 *
 * This method de-activates and stops operation of the USB Hardware.
 * 
 * \bug This method should probably disable the USB Device Interrupts but it
 *   currently doesn't.
 * 
 * \see UsbCoreViaSTM32F4::stop
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::stop(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s()\r\n", __func__);
    
    /* FIXME Should we disable the Device Interrupts here? */

    this->m_usbCore.stop();
}

/***************************************************************************//**
 * @brief USB Device Interrupt Handlers.
 * 
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 * 
 * \warning The OUT Endpoint IRQ needs to be served before the Rx FIFO IRQ. This makes
 *   sure that the Setup Packet is decoded before potential Data OUT Stage Data is
 *   transferred to the device-independent layer.
 *   In other words: On the Control OUT Endpoint, ::usb::stm32f4::OutEndpointViaSTM32F4::dataReceivedDeviceCallback
 *   must not be called before ::usb::stm32f4::CtrlOutEndpointViaSTM32F4::handleSetupDoneIrq as otherwise the
 *   Data OUT Pointers have not been set up.
 * 
 * \see #handleIrq.
 ******************************************************************************/
const
typename UsbDeviceViaSTM32F4::irq_handler_t UsbDeviceViaSTM32F4::m_irq_handler[] = {
    { UsbCoreViaSTM32F4::Interrupt_e::e_Suspend,            &UsbDeviceViaSTM32F4::handleUsbSuspendIrq },
    // { UsbCoreViaSTM32F4::Interrupt_e::e_StartOfFrame,       &UsbDeviceViaSTM32F4::handleStartOfFrame },
    { UsbCoreViaSTM32F4::Interrupt_e::e_OutEndpoint,        &UsbDeviceViaSTM32F4::handleOutEndpointIrq },   // Must be served before e_RxFifoNonEmpty!
    { UsbCoreViaSTM32F4::Interrupt_e::e_RxFifoNonEmpty,     &UsbDeviceViaSTM32F4::handleRxFIFO },
    { UsbCoreViaSTM32F4::Interrupt_e::e_Reset,              &UsbDeviceViaSTM32F4::handleUsbReset },
    { UsbCoreViaSTM32F4::Interrupt_e::e_EnumerationDone,    &UsbDeviceViaSTM32F4::handleEnumerationDone },
    { UsbCoreViaSTM32F4::Interrupt_e::e_InEndpoint,         &UsbDeviceViaSTM32F4::handleInEndpointIrq },
    // { UsbCoreViaSTM32F4::Interrupt_e::e_ConnectorIdStatus,  &UsbDeviceViaSTM32F4::handleConnectorIdStatusChangeIrq },
    // { UsbCoreViaSTM32F4::Interrupt_e::e_EarlySuspend,       &UsbDeviceViaSTM32F4::handleEarlySuspendIrq },
    { UsbCoreViaSTM32F4::Interrupt_e::e_None,               NULL }
};

/***************************************************************************//**
 * @brief USB Device Interrupt Handler.
 * 
 * This method is called by UsbCoreViaSTM32F4::handleIrq to serve the USB Device
 * mode interrupts.
 * 
 * It walks the registered interrupt handlers in #m_irq_handler and calls them
 * if the respective interrupt is active.
 * 
 * @param p_irq Active Interrupts, i.e. contents of the \c GINTSTS Register.
 * @return \c uint32_t Bitmap of served interrupts. Can be used by the caller to
 *   clear the served interrupts in the hardware register.
 ******************************************************************************/
uint32_t
UsbDeviceViaSTM32F4::handleIrq(const uint32_t p_irq) const {
    uint32_t handledIrq = 0;

    USB_PRINTF("--> UsbDeviceViaSTM32F4::%s() p_irq=0x%x\r\n", __func__, p_irq);

    for (const UsbDeviceViaSTM32F4::irq_handler_t *cur = UsbDeviceViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        if (p_irq & cur->m_irq) {
            handledIrq |= cur->m_irq;
            this->m_usbCore.acknowledgeIrq(static_cast<typename UsbCoreViaSTM32F4::Interrupt_t>(cur->m_irq));

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }

    USB_PRINTF("<-- UsbDeviceViaSTM32F4::%s() handledIrq=0x%x\r\n", __func__, handledIrq);

    return handledIrq;
}

/***************************************************************************//**
 * @brief Handles the _USB Suspend_ IRQ
 * 
 * The USB Hardware will trigger this IRQ when the Host has suspended the Bus.
 * 
 * The method will suspend the USB hardware to reduce power consumption.
 * 
 * \bug For USB-powered devices, it would be good to reduce power consumption
 * even more, e.g. by shutting of the µC. This however is not (yet) supported.
 * 
 * \see ::usb::stm32f4::UsbCoreViaSTM32F4::suspendPhy
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleUsbSuspendIrq(void) const {
    if (this->isSuspendableState()) {
        this->m_usbCore.suspendPhy();
        
        /* FIXME Actually suspend µC here */
    }
}

/***************************************************************************//**
 * @brief Initializes the USB Device.
 * 
 * Initializes the USB Device Mode Hardware, e.g. after a Reset.
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::initialize(void) const {
    this->disableInterrupts();

#if 0
    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        if (this->m_inEndpoints[idx] != NULL) {
            this->m_inEndpoints[idx]->disable();
        }
    }
#endif

    for (unsigned idx = 0; idx < this->m_maxOutEndpoints; idx++) {
        if (this->m_outEndpoints[idx] != NULL) {
            this->m_outEndpoints[idx]->disable();
        }
    }

    // this->disconnect();
}

/***************************************************************************//**
 * @brief Set up the Tx FIFOs for all Endpoints.
 * 
 * This method is called during Hardware initialization to set up the Tx FIFOs
 * for all Endpoints.
 * 
 * There is a 1:1 mapping between IN Endpoint Number and Tx FIFO number, i.e.
 * IN Endpoint Zero will use the first FIFO, IN Endpoint One the second FIFO
 * etc.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setupTxFifos(void) const {
    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        if (this->m_inEndpoints[idx] != NULL) {
            this->m_inEndpoints[idx]->setupTxFifoNumber(idx);
            this->setupTxFifo(*this->m_inEndpoints[idx]);
        } else {
            this->m_usbCore.setupTxFifo(idx, 0);
        }
    }
}

/***************************************************************************//**
 * @brief Set up Tx FIFO for an IN Endpoint.
 * 
 * This method sets up the given IN Endpoint's Tx FIFO.
 * 
 * @param p_endpoint Reference to IN Endpoint.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setupTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbCore.setupTxFifo(p_endpoint.getEndpointNumber(), p_endpoint.getFifoSzInWords());
}

/***************************************************************************//**
 * @brief Flush the Tx FIFO of an IN Endpoint.
 * 
 * This method flushes up the given IN Endpoint's Tx FIFO by calling
 * UsbCoreViaSTM32F4::flushTxFifo internally.
 * 
 * @param p_endpoint Reference to IN Endpoint.
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::flushTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbCore.flushTxFifo(p_endpoint.getEndpointNumber());
}

/***************************************************************************//**
 * @brief Read enumerated speed from USB Hardware.
 * 
 * Reads the \c ENUMSPD bits from the \c DSTS register and decodes it into an
 * enumerated USB Speed.
 * 
 * \warning This method will only yield deterministic results after the USB
 * enumeration is complete.
 * 
 * \see ::usb::stm32f4::UsbDeviceViaSTM32F4::handleEnumerationDone.
 * 
 * \return Enumerated Device Speed.
 ******************************************************************************/
::usb::UsbHwDevice::DeviceSpeed_e
UsbDeviceViaSTM32F4::getEnumeratedSpeed(void) const {
    ::usb::UsbHwDevice::DeviceSpeed_e enumeratedUsbSpeed = ::usb::UsbHwDevice::DeviceSpeed_e::e_UsbUnknown;

    /*
     * The CPU Reference Manual lists the Bit Combination 0b01 and 0b10 as reserved. I've
     * however noticed in some of my tests that the DTS_ENUMSPD bits sometimes do come up
     * as 0b01.
     * 
     * The mbed Codebase has a driver for the same core, but on STM32F1, which #defines
     * essntially the below.
     *
     * The same thing applies for the USB OTG Driver that is part of the ST USB OTG Demo,
     * see STSW-STM32046 on st.com.
     *
     *  #define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ      0b00
     *  #define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ      0b01
     *  #define DSTS_ENUMSPD_LS_PHY_6MHZ                0b10
     *  #define DSTS_ENUMSPD_FS_PHY_48MHZ               0b11
     *
     * Also, the same driver(s) assumes the following mapping:
     *
     *  DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ  -> USB_SPEED_HIGH
     *  DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ  -> USB_SPEED_FULL
     *  DSTS_ENUMSPD_FS_PHY_48MHZ           -> USB_SPEED_FULL
     *  DSTS_ENUMSPD_LS_PHY_6MHZ            -> USB_SPEED_LOW
     */
    enum class EnumeratedSpeed_e : uint8_t {
        e_HighSpeed                 = 0b00,
        e_FullSpeed_30MHz_or_60MHz  = 0b01,
        e_LowSpeed                  = 0b10,
        e_FullSpeed                 = 0b11
    };

    EnumeratedSpeed_e emumeratedDeviceSpeed = static_cast<EnumeratedSpeed_e>((this->m_usbDevice->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos);

    switch(emumeratedDeviceSpeed) {
    case EnumeratedSpeed_e::e_FullSpeed_30MHz_or_60MHz: // TODO Why is this needed?
        /*
         * This case was not needed until a rework of the stm32-common architecture.
         * There seems to be an issue with timing, i.e. how fast / slow the IRQs
         * leading to this method being called are served.
         *
         * The PLL and GPIO Settings were checked and seem to be the same as before.
         */
    case EnumeratedSpeed_e::e_FullSpeed:
        enumeratedUsbSpeed = e_UsbFullSpeed;
        break;
    default:
        /* FIXME USB Low or High Speed Mode not yet implemented */
        assert(false);
        break;
    }

    return enumeratedUsbSpeed;
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00 */
