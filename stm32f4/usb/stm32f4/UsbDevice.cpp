/*-
 * $Copyright$
-*/

#ifndef _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00
#define	_USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00

#include "UsbCore.hpp"
#include "UsbDevice.hpp"
#include "InEndpoint.hpp"
#include "OutEndpoint.hpp"

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
UsbDeviceBase::UsbDeviceBase(intptr_t p_deviceAddr, const void * const p_configurationDescriptor,
  const ::usb::UsbStringDescriptors_t &p_stringDescriptors, const DeviceSpeed_e p_deviceSpeed)
  : m_usbDevice(reinterpret_cast<USB_OTG_DeviceTypeDef *>(p_deviceAddr)),
    m_maxConfigurations(1),
    m_deviceDescriptor {
        sizeof(m_deviceDescriptor),                                     /* m_bLength */
        ::usb::UsbDescriptorTypeId_t::e_Device,                         /* m_bDescriptorType */
        { 0x00, 0x02 },                                                 /* m_bLength */
        0x00,                                                           /* m_bDeviceClass */
        0x00,                                                           /* m_bDeviceSubClass */
        0x00,                                                           /* m_bDeviceProtocol */
        static_cast<uint8_t>(p_deviceSpeed == e_UsbFullSpeed ? 64 : 0), /* m_bMaxPacketSize0 */
        { 0xad, 0xde },                                                 /* m_idVendor */
        { 0xef, 0xbe },                                                 /* m_idProduct */
        { 0xfe, 0xca },                                                 /* m_bcdDevice */
        ::usb::UsbStringDescriptorId_t::e_StrDesc_Manufacturer,         /* e_iManufacturer */
        ::usb::UsbStringDescriptorId_t::e_StrDesc_Product,              /* e_iProduct */
        ::usb::UsbStringDescriptorId_t::e_StrDesc_SerialNumber,         /* e_iSerialNumber */
        m_maxConfigurations                                             /* e_bNumConfigurations */
    },
    m_deviceQualifierDescriptor(m_deviceDescriptor),
    m_configurationDescriptor(p_configurationDescriptor),
    m_stringDescriptors(p_stringDescriptors),
    m_deviceSpeed(p_deviceSpeed),
    m_deviceStatus { 0x1 }
{

}

/*******************************************************************************
 *
 ******************************************************************************/
UsbDeviceBase::~UsbDeviceBase() {

}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::registerEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] == NULL);

    this->m_inEndpoints[p_endpointNumber] = &p_endpoint;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::unregisterEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint) {
    assert(p_endpointNumber < this->m_maxInEndpoints);
    assert(this->m_inEndpoints[p_endpointNumber] == &p_endpoint);

    this->m_inEndpoints[p_endpointNumber] = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::registerEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] == NULL);

    this->m_outEndpoints[p_endpointNumber] = &p_endpoint;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::unregisterEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint) {
    assert(p_endpointNumber < this->m_maxOutEndpoints);
    assert(this->m_outEndpoints[p_endpointNumber] == &p_endpoint);

    this->m_outEndpoints[p_endpointNumber] = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::setupDeviceSpeed(const DeviceSpeed_e p_speed) const {
    this->m_usbDevice->DCFG &= ~USB_OTG_DCFG_DSPD;
    this->m_usbDevice->DCFG |= p_speed & USB_OTG_DCFG_DSPD;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disableInterrupts(void) const {
    this->m_usbDevice->DIEPMSK      = 0u;
    this->m_usbDevice->DOEPMSK      = 0u;
    this->m_usbDevice->DAINTMSK     = 0u;
    this->m_usbDevice->DIEPEMPMSK   = 0u;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disconnect(void) const {
    this->m_usbDevice->DCTL |= USB_OTG_DCTL_SDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::connect(void) const {
    this->m_usbDevice->DCTL &= ~USB_OTG_DCTL_SDIS;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::handleInEndpointIrq(void) const {
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
UsbDeviceBase::handleOutEndpointIrq(void) const {
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
UsbDeviceBase::handleDisconnectIrq(void) const {
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::handleEnumerationDone(void) const {
	/*
	 * As per STM32F4 Datasheet, we could read the enumerated speed here from
	 * the DSTS register.
	 */
	// assert((this->m_usbDevice->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) == (3 << USB_OTG_DSTS_ENUMSPD_Pos));

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
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::handleUsbReset(void) const {
    this->setAddress(0);

    /*
     * Endpoint Initialization upon USB Reset as per STM32F4 Datasheet:
     *
     *  1. Set NAK for all OUT Endpoints
     */
    for (unsigned idx = 0; idx < this->m_maxOutEndpoints; idx++) {
        OutEndpoint *endpt = this->m_outEndpoints[idx];
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
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::handleStartOfFrame(void) const {

}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::handleRxFIFO(const uint32_t p_rxStatus) const {
	uint32_t endpointNum = p_rxStatus & USB_OTG_GRXSTSP_EPNUM;
	uint32_t numBytes = (p_rxStatus & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;

	switch ((p_rxStatus & USB_OTG_GRXSTSP_PKTSTS) >> USB_OTG_GRXSTSP_PKTSTS_Pos) {
	case 0x01: /* Global OUT NACK */
		break;
	case 0x02: /* OUT Data Packet Received */
		break;
	case 0x03: /* OUT Txfr Completed */
		break;
	case 0x04: /* Setup Transaction Complete */
		break;
	case 0x06: /* Setup Data Packet Received */
		if (this->m_outEndpoints[endpointNum] != NULL) {
			this->m_outEndpoints[endpointNum]->handleRxData(numBytes);
		}
		break;
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
bool
UsbDeviceBase::isSuspendableState(void) const {
    /* FIXME Actually determine suspendable state here */
    return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disableEndpointIrq(const InEndpoint &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::enableEndpointIrq(const InEndpoint &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disableEndpointFifoIrq(const InEndpoint &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK &= ~(1 << p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::enableEndpointFifoIrq(const InEndpoint &p_endpoint) const {
    this->m_usbDevice->DIEPEMPMSK |= (1 << p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disableEndpointIrq(const OutEndpoint &p_endpoint) const {
    this->disableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::enableEndpointIrq(const OutEndpoint &p_endpoint) const {
    this->enableEndpointIrq(p_endpoint.getEndpointNumber(), true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const {
    this->m_usbDevice->DAINTMSK &= ~(1 << (p_endpointNumber + (p_isOutEndpoint ? 16 : 0)));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::enableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const {
    this->m_usbDevice->DAINTMSK |= 1 << (p_endpointNumber + (p_isOutEndpoint ? 16 : 0));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::setAddress(const uint8_t p_address) const {
    assert(p_address < 128);

    this->m_usbDevice->DCFG &= ~USB_OTG_DCFG_DAD;
    this->m_usbDevice->DCFG |= (p_address << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD_Msk;

    /* Acknowledge the setAddress() command on the Default Ctrl Endpoint */
    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(NULL, 0, 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::setConfiguration(const uint8_t /* p_configuration */) const {
    /* Acknowledge the setConfiguration() command on the Default Ctrl Endpoint */
    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(NULL, 0, 0);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getStatus(const uint8_t p_len) const {
    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(reinterpret_cast<const uint8_t *>(&this->m_deviceStatus), sizeof(this->m_deviceStatus), p_len);
}


/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getDescriptor(const uint16_t p_descriptor, const size_t p_len) const {
    const UsbDescriptorTypeId_t descriptorType = static_cast<UsbDescriptorTypeId_t>((p_descriptor >> 8) & 0x07);
    uint8_t                     descriptorId = p_descriptor & 0xFF;

    switch (descriptorType) {
    case e_Device:
        this->getDeviceDescriptor(descriptorId, p_len);
        break;
    case e_String:
        this->getStringDescriptor(descriptorId, p_len);
        break;
    case e_Configuration:
        this->getConfigurationDescriptor(descriptorId, p_len);
        break;
    case e_DeviceQualifier:
        this->getDeviceQualifierDescriptor(descriptorId, p_len);
        break;
    case e_Interface:
    case e_Endpoint:
    case e_OtherSpeedConfig:
        /* FIXME Not yet implemented */
        assert(false);
        break;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getDeviceDescriptor(const uint8_t p_descriptorId, const size_t p_len) const {
    assert(p_descriptorId == 0);

    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(reinterpret_cast<const uint8_t *>(&this->m_deviceDescriptor), sizeof(this->m_deviceDescriptor), p_len);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getDeviceQualifierDescriptor(const uint8_t p_descriptorId, const size_t p_len) const {
    assert(p_descriptorId == 0);

    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(reinterpret_cast<const uint8_t *>(&this->m_deviceQualifierDescriptor), sizeof(this->m_deviceQualifierDescriptor), p_len);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getConfigurationDescriptor(const uint8_t p_descriptorId, const size_t p_len) const {
    assert(p_descriptorId == 0);

    assert(this->m_inEndpoints[0] != NULL);
    this->m_inEndpoints[0]->write(reinterpret_cast<const uint8_t *>(this->m_configurationDescriptor), p_len, p_len);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbDeviceBase::getStringDescriptor(const uint8_t p_descriptorId, const size_t p_len) const {
    UsbStringDescriptorId_t stringDescriptor = static_cast<UsbStringDescriptorId_t>(p_descriptorId);

    if (p_descriptorId >= e_StrDesc_Max) {
        goto out;
    }

    assert(this->m_inEndpoints[0] != NULL);

    switch (stringDescriptor) {
    case e_StrDesc_LanguageId: {
        static uint8_t  buffer[64];
        UsbLangId_t *   cur = reinterpret_cast<UsbLangId_t *>(&buffer[2]);
        unsigned        idx;

        buffer[1] = e_String;

        for (idx = 0; idx < this->m_stringDescriptors.m_stringDescriptorTable.m_languageIds.m_numLanguages; idx++, cur++) {
            * cur = this->m_stringDescriptors.m_stringDescriptorTable.m_languageIds.m_langIds[idx];
        }

        buffer[0] = 2 + idx * sizeof(UsbLangId_t);

        this->m_inEndpoints[0]->write(buffer, buffer[0], p_len);
    } break;
    case e_StrDesc_Manufacturer:
    case e_StrDesc_Product:
    case e_StrDesc_SerialNumber:
    case e_StrDesc_Configuration:
    case e_StrDesc_Interface:
        this->m_inEndpoints[0]->writeString(this->m_stringDescriptors.m_stringDescriptors[stringDescriptor], p_len);
        break;
    default:
        break;
    }

out:
    return;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::start(void) const {
    this->setupDeviceSpeed(this->m_deviceSpeed);
    this->m_usbCore.start();

    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        if (this->m_inEndpoints[idx] != NULL) {
            this->flushTxFifo(*(this->m_inEndpoints[idx]));
        }
    }
    
    this->m_usbCore.enableInterrupt(UsbCoreT::e_Suspend);
    this->m_usbCore.enableInterrupt(UsbCoreT::e_StartOfFrame);
    this->m_usbCore.enableInterrupt(UsbCoreT::e_Reset);
    this->m_usbCore.enableInterrupt(UsbCoreT::e_EnumerationDone);
    this->m_usbCore.enableInterrupt(UsbCoreT::e_SessionRequest);
    this->m_usbCore.enableInterrupt(UsbCoreT::e_OnTheGo);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::stop(void) const {
    this->m_usbCore.stop();
}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
template<typename UsbCoreT>
const
typename UsbDeviceViaUsbCoreT<UsbCoreT>::irq_handler_t UsbDeviceViaUsbCoreT<UsbCoreT>::m_irq_handler[] = {
    { USB_OTG_GINTSTS_USBSUSP, &UsbDeviceViaUsbCoreT<UsbCoreT>::handleUsbSuspend },
    { USB_OTG_GINTSTS_SOF, &UsbDeviceBase::handleStartOfFrame },
    { USB_OTG_GINTSTS_RXFLVL, &UsbDeviceViaUsbCoreT<UsbCoreT>::handleRxFIFO },
    { USB_OTG_GINTSTS_USBRST, &UsbDeviceBase::handleUsbReset },
    { USB_OTG_GINTSTS_ENUMDNE, &UsbDeviceViaUsbCoreT<UsbCoreT>::handleEnumerationDone },
    { USB_OTG_GINTSTS_IEPINT, &UsbDeviceBase::handleInEndpointIrq },
    { USB_OTG_GINTSTS_OEPINT, &UsbDeviceBase::handleOutEndpointIrq },
    { USB_OTG_GINTSTS_DISCINT, &UsbDeviceBase::handleDisconnectIrq },
    { 0, NULL }
};

template<typename UsbCoreT>
uint32_t
UsbDeviceViaUsbCoreT<UsbCoreT>::handleIrq(const uint32_t p_irq) const {
    uint32_t handledIrq = 0;

    for (const UsbDeviceViaUsbCoreT<UsbCoreT>::irq_handler_t *cur = UsbDeviceViaUsbCoreT<UsbCoreT>::m_irq_handler; cur->m_irq != 0; cur++) {
        if (p_irq & cur->m_irq) {
            handledIrq |= cur->m_irq;
            this->m_usbCore.acknowledgeIrq(static_cast<typename UsbCoreT::Interrupt_t>(cur->m_irq));

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }

    return handledIrq;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::handleEnumerationDone(void) const {
	UsbDeviceBase::handleEnumerationDone();

	this->m_usbCore.setUsbTurnAroundTime(5); // Value 5 is taken from ST Example Code

	this->m_usbCore.enableInterrupt(UsbCoreT::e_InEndpoint);
	this->m_usbCore.enableInterrupt(UsbCoreT::e_OutEndpoint);
	this->m_usbCore.enableInterrupt(UsbCoreT::e_Disconnected);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::handleUsbSuspend(void) const {
    if (this->isSuspendableState()) {
        this->m_usbCore.suspendPhy();
        
        /* FIXME Actually suspend µC here */
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::handleRxFIFO(void) const {
	uint32_t rxStatus = this->m_usbCore.getRxStatus();

	UsbDeviceBase::handleRxFIFO(rxStatus);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::initialize(void) const {
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
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::setupTxFifos(void) const {
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
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::setupTxFifo(const InEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> > &p_endpoint) const {
    this->m_usbCore.setupTxFifo(p_endpoint.getEndpointNumber(), p_endpoint.getFifoSzInWords());
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
void
UsbDeviceViaUsbCoreT<UsbCoreT>::flushTxFifo(const InEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> > &p_endpoint) const {
    this->m_usbCore.flushTxFifo(p_endpoint.getEndpointNumber());
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
UsbDeviceViaUsbCoreT<UsbCoreT>::UsbDeviceViaUsbCoreT(UsbCoreT &p_usbCore, const void * const p_configurationDescriptor,
  const ::usb::UsbStringDescriptors_t &p_stringDescriptors, const DeviceSpeed_e p_deviceSpeed /* = e_UsbFullSpeed */)
  : UsbDeviceBase(p_usbCore.getBaseAddr() + USB_OTG_DEVICE_BASE, p_configurationDescriptor, p_stringDescriptors, p_deviceSpeed),
  m_usbCore(p_usbCore) {
    m_usbCore.setupMode(UsbCoreT::e_UsbDevice);
    m_usbCore.registerDevice(*this);

    this->m_usbCore.enableInterrupt(UsbCoreT::e_ModeMismatch);

    this->initialize();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT>
UsbDeviceViaUsbCoreT<UsbCoreT>::~UsbDeviceViaUsbCoreT() {
    m_usbCore.unregisterDevice(*this);
}

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBDEVICE_CPP_5ce6d113_0a23_4757_984c_2f535cf9de00 */
