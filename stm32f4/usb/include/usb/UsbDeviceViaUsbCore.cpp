/*-
 * $Copyright$
-*/

#include <usb/UsbDevice.hpp>
#include <usb/InEndpoint.hpp>
#include <usb/OutEndpoint.hpp>

namespace usb {
    namespace stm32f4 {

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

    } /* namespace stm32f4 */
} /* namespace usb */

/*******************************************************************************
 *
 ******************************************************************************/
