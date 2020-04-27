/*-
 * $Copyright$
-*/

#ifndef _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00
#define	_USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00

#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbCoreViaSTM32F4.hpp>

#include <usb/InEndpointViaSTM32F4.hpp>
#include <usb/OutEndpointViaSTM32F4.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
UsbDeviceViaSTM32F4::UsbDeviceViaSTM32F4(UsbCoreViaSTM32F4 &p_usbCore, const DeviceSpeed_e p_deviceSpeed /* = DeviceSpeed_e::e_UsbFullSpeed */)
  : UsbHwDevice(p_deviceSpeed),
    m_usbCore(p_usbCore),
    m_usbDevice(reinterpret_cast<USB_OTG_DeviceTypeDef *>(p_usbCore.getBaseAddr() + USB_OTG_DEVICE_BASE)),
    m_deviceStatus { 0x1 }
{
    m_usbCore.setupMode(UsbCoreViaSTM32F4::e_UsbDevice);
    m_usbCore.registerDevice(*this);

    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_ModeMismatch);

    this->initialize();
}

/*******************************************************************************
 *
 ******************************************************************************/
UsbDeviceViaSTM32F4::~UsbDeviceViaSTM32F4() {
    m_usbCore.unregisterDevice(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::registerEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] == NULL);

    this->m_inEndpoints[p_endpointNumber] = &p_endpoint;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::unregisterEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] == &p_endpoint);

    this->m_inEndpoints[p_endpointNumber] = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::registerEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] == NULL);

    this->m_outEndpoints[p_endpointNumber] = &p_endpoint;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::unregisterEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] == &p_endpoint);

    this->m_outEndpoints[p_endpointNumber] = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setupDeviceSpeed(const DeviceSpeed_e p_speed) const {
    this->m_usbDevice->DCFG &= ~USB_OTG_DCFG_DSPD;
    this->m_usbDevice->DCFG |= p_speed & USB_OTG_DCFG_DSPD;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableInterrupts(void) const {
    this->m_usbDevice->DIEPMSK      = 0u;
    this->m_usbDevice->DOEPMSK      = 0u;
    this->m_usbDevice->DAINTMSK     = 0u;
    this->m_usbDevice->DIEPEMPMSK   = 0u;
}

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

/*******************************************************************************
 *
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

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleOutEndpointIrq(void) const {
    uint16_t irqs = (this->m_usbDevice->DAINT & USB_OTG_DAINT_OEPINT_Msk) >> USB_OTG_DAINT_OEPINT_Pos;

    for (unsigned idx = 0; idx < this->m_maxOutEndpoints; idx++) {
        if (irqs & (1 << idx)) {
            if (this->m_outEndpoints[idx] != NULL) {
                this->m_outEndpoints[idx]->handleIrq();
            }
        }
    }
}

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
UsbDeviceViaSTM32F4::handleUsbSuspendIrq(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s()\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleEarlySuspendIrq(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s()\r\n", __func__);  
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleEnumerationDone(void) const {
	/*
	 * As per STM32F4 Datasheet, we could read the enumerated speed here from
	 * the DSTS register.
	 */
	assert((this->m_usbDevice->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) == (3 << USB_OTG_DSTS_ENUMSPD_Pos));

	/*
	 * Because we're a FullSpeed device, we must set the max. packet size for
	 * control IN packets to 64 Bytes
	 */
	assert(this->m_inEndpoints[0] != NULL);
	this->m_inEndpoints[0]->setPacketSize(64);

	this->m_usbDevice->DCTL |= USB_OTG_DCTL_CGINAK;

	/* Enable Control Endpoint 0 */
#if 0
	assert(this->m_inEndpoints[0] != NULL);
	this->m_inEndpoints[0]->enable();
#endif

	assert(this->m_outEndpoints[0] != NULL);
	this->m_outEndpoints[0]->enable();

	this->m_usbCore.setUsbTurnAroundTime(5); // Value 5 is taken from ST Example Code

	this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_InEndpoint);
	this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_OutEndpoint);

    // USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device enumeration done.\r\n", __func__);
}

/*******************************************************************************
 *
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
#if 0
    assert(this->m_inEndpoints[0] != NULL);
    this->enableEndpointIrq(*(this->m_inEndpoints[0]));
#endif

    assert(this->m_outEndpoints[0] != NULL);
    this->enableEndpointIrq(*(this->m_outEndpoints[0]));

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

    // These are programmed during construction of the C++ Objects (Usb Core, Endpoints)

    /* 4. Program the endpoint-specific registers for control OUT endpoint 0
     *    to receive a SETUP packet
     */
    this->m_outEndpoints[0]->enable();

    USB_PRINTF("UsbDeviceViaSTM32F4::%s() USB Device reset complete.\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleStartOfFrame(void) const {

}

/*******************************************************************************
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

        USB_PRINTF("UsbDeviceViaSTM32F4::%s(): OUT Packet Received -- numBytes=%d, endpointNum = %d, dataPID=%x\r\n", __func__, numBytes, endpointNum, dataPID);

		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->handleRxData(numBytes, dataPID);
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

        USB_PRINTF("UsbDeviceViaSTM32F4::%s(): OUT Txfr Completed -- endpointNum = %i, dataPID=%x\r\n", __func__, endpointNum, dataPID);
        
		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->handleOutTransferComplete();
		}
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
        USB_PRINTF("UsbDeviceViaSTM32F4::%s(): SETUP Transaction complete -- numBytes=%d, endpointNum = %d, dataPID=%x\r\n", __func__, numBytes, endpointNum, dataPID);

        assert(endpointNum == 0);
        assert(numBytes == 0);
        assert(dataPID == 0);

		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->handleSetupComplete();
		}
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

        USB_PRINTF("UsbDeviceViaSTM32F4::%s(): SETUP Packet numBytes=%d, endpointNum = %d, dataPID=%x\r\n", __func__, numBytes, endpointNum, dataPID);

		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->handleSetupData(numBytes);
		}
		break;
    default:
        assert(false);
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
bool
UsbDeviceViaSTM32F4::isSuspendableState(void) const {
    /* TODO Actually determine suspendable state here */
    return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK &= ~(1 << p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK |= (1 << p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::enableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const {
    this->m_usbDevice->DAINTMSK &= ~(1 << (p_endpointNumber + (p_isOutEndpoint ? 16 : 0)));
}

/*******************************************************************************
 *
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

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(USB_DEBUG)
void
UsbDeviceViaSTM32F4::debugPrint(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s(): DAINTMSK=%x, \r\n", __func__, this->m_usbDevice->DAINTMSK);
}
#endif /* defined(USB_DEBUG) */

/*******************************************************************************
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
    
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_Suspend);
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_StartOfFrame);
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_Reset);
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_EnumerationDone);
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_SessionRequest);
    this->m_usbCore.enableInterrupt(UsbCoreViaSTM32F4::e_OnTheGo);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::stop(void) const {
    USB_PRINTF("UsbDeviceViaSTM32F4::%s()\r\n", __func__);
    
    this->m_usbCore.stop();
}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
const
typename UsbDeviceViaSTM32F4::irq_handler_t UsbDeviceViaSTM32F4::m_irq_handler[] = {
    { USB_OTG_GINTSTS_USBSUSP,  &UsbDeviceViaSTM32F4::handleUsbSuspend },
    { USB_OTG_GINTSTS_SOF,      &UsbDeviceViaSTM32F4::handleStartOfFrame },
    { USB_OTG_GINTSTS_RXFLVL,   &UsbDeviceViaSTM32F4::handleRxFIFO },
    { USB_OTG_GINTSTS_USBRST,   &UsbDeviceViaSTM32F4::handleUsbReset },
    { USB_OTG_GINTSTS_ENUMDNE,  &UsbDeviceViaSTM32F4::handleEnumerationDone },
    { USB_OTG_GINTSTS_IEPINT,   &UsbDeviceViaSTM32F4::handleInEndpointIrq },
    { USB_OTG_GINTSTS_OEPINT,   &UsbDeviceViaSTM32F4::handleOutEndpointIrq },
    { USB_OTG_GINTSTS_CIDSCHG,  &UsbDeviceViaSTM32F4::handleConnectorIdStatusChangeIrq },
    { USB_OTG_GINTSTS_ESUSP,    &UsbDeviceViaSTM32F4::handleEarlySuspendIrq },
    { USB_OTG_GINTSTS_USBSUSP,  &UsbDeviceViaSTM32F4::handleUsbSuspendIrq },
    { 0, NULL }
};

uint32_t
UsbDeviceViaSTM32F4::handleIrq(const uint32_t p_irq) const {
    uint32_t handledIrq = 0;

    for (const UsbDeviceViaSTM32F4::irq_handler_t *cur = UsbDeviceViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        if (p_irq & cur->m_irq) {
            handledIrq |= cur->m_irq;
            this->m_usbCore.acknowledgeIrq(static_cast<typename UsbCoreViaSTM32F4::Interrupt_t>(cur->m_irq));

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }

    return handledIrq;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::handleUsbSuspend(void) const {
    // USB_PRINTF("%s()\r\n", __func__);

    if (this->isSuspendableState()) {
        this->m_usbCore.suspendPhy();
        
        /* FIXME Actually suspend µC here */
    }
}

/*******************************************************************************
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
/*******************************************************************************
 *
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

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::setupTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbCore.setupTxFifo(p_endpoint.getEndpointNumber(), p_endpoint.getFifoSzInWords());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceViaSTM32F4::flushTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const {
    this->m_usbCore.flushTxFifo(p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00 */
