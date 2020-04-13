/*-
 * $Copyright$
-*/

#ifndef _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09
#define	_USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09

#include "UsbCore.hpp"
#include "UsbDevice.hpp"

#include <compat/unistd.h>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
UsbCore::UsbCore(USB_OTG_GlobalTypeDef * const p_usbCore, intptr_t p_usbPowerCtrl, const uint32_t p_rxFifoSzInWords)
  : m_usbCore(p_usbCore), m_usbPwrCtrl(reinterpret_cast<PowerAndClockGatingControl_t *>(p_usbPowerCtrl)), m_rxFifoSzInWords(p_rxFifoSzInWords) {
}

/*******************************************************************************
 *
 ******************************************************************************/
UsbCore::~UsbCore() {
}


/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::initialize() const {
    this->reset();

    this->startPhy();
    this->startTransceiver();

    /*
     * Let USB Core settle for a bit. If not, then the session request IRQ will
     * fire immediately.
     */
    usleep(25000);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::terminate() const {
    this->reset();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::registerDevice(UsbDeviceViaUsbCoreT<UsbCore> &p_device) {
    assert(this->m_usbDevice == NULL);
    
    m_usbDevice = &p_device;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::unregisterDevice(UsbDeviceViaUsbCoreT<UsbCore> &p_device) {
    assert(this->m_usbDevice == &p_device);
    
    this->m_usbDevice = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const {
    uint16_t offset;

    /* FIXME: Host mode not (yet) supported */
    assert((this->m_usbCore->GUSBCFG & USB_OTG_GUSBCFG_FDMOD) != 0);

    if (p_endpoint == 0) {
        /* Calculate EP0 FIFO Offset based on Global Rx FIFO Length (which is configured in words) */
        this->m_usbCore->DIEPTXF0_HNPTXFSIZ = (p_fifoSzInWords << 16) | ((this->m_usbCore->GRXFSIZ) & 0x0000FFFF);
    } else {
        if (p_endpoint == 1) {
            offset = (this->m_usbCore->DIEPTXF0_HNPTXFSIZ & 0x0000ffff) + ((this->m_usbCore->DIEPTXF0_HNPTXFSIZ >> 16) & 0x0000FFFF);
        } else {
            offset = (this->m_usbCore->DIEPTXF[p_endpoint - 2] & 0x0000ffff) + ((this->m_usbCore->DIEPTXF[p_endpoint - 2] >> 16) & 0x0000FFFF);
        }
        this->m_usbCore->DIEPTXF[p_endpoint - 1] = (p_fifoSzInWords << 16) | (offset & 0x0000FFFF);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::reset(void) const {
    this->performReset(USB_OTG_GRSTCTL_CSRST);

    usleep(25);

    this->disableInterrupt();
    this->stopPhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::performReset(const uint32_t p_reset) const {
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));

    this->m_usbCore->GRSTCTL |= p_reset;

    while (this->m_usbCore->GRSTCTL & p_reset);
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::setRxFifoSz(const uint16_t p_rxFifoSzInWords) const {
    // Minimum Rx FIFO size is 16, maximum size is 256 (all in words)
    assert((m_rxFifoSzInWords > 16) && (m_rxFifoSzInWords <= 256));

    this->m_usbCore->GRXFSIZ = p_rxFifoSzInWords;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::flushRxFifo(void) const {
    this->performReset(USB_OTG_GRSTCTL_RXFFLSH);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::flushTxFifo(const uint8_t p_fifo) const {
    assert(p_fifo <= 16);
    
    this->m_usbCore->GRSTCTL &= ~USB_OTG_GRSTCTL_TXFNUM;
    this->m_usbCore->GRSTCTL |= ((p_fifo << 6) & USB_OTG_GRSTCTL_TXFNUM);

    this->performReset(USB_OTG_GRSTCTL_TXFFLSH);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::enableInterrupt(void) const {
    this->m_usbCore->GINTSTS = 0xffffffff;
    this->m_usbCore->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::enableInterrupt(const Interrupt_t p_irq) const {
    this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
    this->m_usbCore->GINTMSK |= static_cast<uint32_t>(p_irq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::disableInterrupt(void) const {
    this->m_usbCore->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::disableInterrupt(const Interrupt_t p_irq) const {
    this->m_usbCore->GINTMSK &= ~(static_cast<uint32_t>(p_irq));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::startPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL &= ~USB_OTG_PCGCCTL_PHYSUSP;
    this->resumePhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::resumePhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    
    while (this->m_usbPwrCtrl->PCGCCTL  & USB_OTG_PCGCCTL_PHYSUSP);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::suspendPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL |= (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    while (!(this->m_usbPwrCtrl->PCGCCTL & USB_OTG_PCGCCTL_PHYSUSP));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::stopPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL |= USB_OTG_PCGCCTL_PHYSUSP;
    this->suspendPhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::startTransceiver(void) const {
    this->m_usbCore->GCCFG |= USB_OTG_GCCFG_PWRDWN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::stopTransceiver(void) const {
    this->m_usbCore->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::setupMode(const DeviceMode_e p_mode) {
    this->m_mode = p_mode;

    this->setupModeInHw(this->m_mode);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::setupModeInHw(const DeviceMode_e p_mode) const {
    switch (p_mode) {
    case e_UsbDevice:
        this->m_usbCore->GCCFG |= USB_OTG_GCCFG_VBUSBSEN;
        this->m_usbCore->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

        this->startPhy();
        this->startTransceiver();

        /*
         * Let USB Core settle for a bit. If not, then the session request IRQ will
         * fire immediately.
         */
        usleep(25000);

        this->flushRxFifo();
        this->flushTxFifo(0x10); // Flush all Tx FIFOs

        this->setRxFifoSz(this->m_rxFifoSzInWords);
        break;
    case e_UsbHost:
        this->m_usbCore->GCCFG |= USB_OTG_GCCFG_VBUSASEN;
        this->m_usbCore->GUSBCFG &= ~USB_OTG_GUSBCFG_FDMOD;

        assert(0); // Host Mode not (yet?) supported
        break;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::start(void) const {
    this->startPhy();
    this->startTransceiver();

    /*
     * Let USB Core settle for a bit. If not, then the session request IRQ will
     * fire immediately.
     */
    usleep(25000);

    this->enableInterrupt();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::stop(void) const {
    this->disableInterrupt();

    this->stopTransceiver();
    this->stopPhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::handleSessionEnd(void) const {
    if (this->m_mode == e_UsbDevice) {
        assert(this->m_usbDevice != NULL);
        
        this->m_usbDevice->stop();
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of OTG interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
const
UsbCore::irq_handler_t UsbCore::m_otgirq_handler[] = {
    { USB_OTG_GOTGINT_SEDET,    &UsbCore::handleSessionEnd },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::handleOtgIrq(void) const {
    uint32_t irqStatus = this->m_usbCore->GOTGINT;

    for (const UsbCore::irq_handler_t *cur = UsbCore::m_otgirq_handler; cur->m_irq != 0; cur++) {
        if (irqStatus & cur->m_irq) {
            this->m_usbCore->GOTGINT = cur->m_irq;
            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::handleModeMismatchIrq(void) const {
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::handleSessionRequest(void) const {

}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
const
UsbCore::irq_handler_t UsbCore::m_irq_handler[] = {
    { USB_OTG_GINTSTS_SRQINT,   &UsbCore::handleSessionRequest },
    { USB_OTG_GINTSTS_MMIS,     &UsbCore::handleModeMismatchIrq },
    { USB_OTG_GINTSTS_OTGINT,   &UsbCore::handleOtgIrq },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::handleIrq(void) const {
    uint32_t irqStatus = this->m_usbCore->GINTSTS;
    uint32_t handledIrq = 0;

    this->disableInterrupt();

    for (const UsbCore::irq_handler_t *cur = UsbCore::m_irq_handler; cur->m_irq != 0; cur++) {
        if (irqStatus & cur->m_irq) {
            handledIrq |= cur->m_irq;
            this->acknowledgeIrq(static_cast<typename UsbCore::Interrupt_t>(cur->m_irq));

            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }
    
    if (this->m_usbDevice != NULL) {
        handledIrq |= this->m_usbDevice->handleIrq(irqStatus);
    }
    
    this->m_usbCore->GINTSTS = handledIrq;

    this->enableInterrupt();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::acknowledgeIrq(const Interrupt_t p_irq) const {
	this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCore::setUsbTurnAroundTime(const uint8_t p_turnaroundTime) const {
	this->m_usbCore->GUSBCFG |= (p_turnaroundTime << USB_OTG_GUSBCFG_TRDT_Pos) & USB_OTG_GUSBCFG_TRDT_Msk;
}

/*******************************************************************************
 *
 ******************************************************************************/
uint32_t
UsbCore::getRxStatus(void) const {
	return this->m_usbCore->GRXSTSP;
}
/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09 */
  
