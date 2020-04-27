/*-
 * $Copyright$
-*/

#ifndef _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09
#define	_USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09

#include <usb/UsbCoreViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbTypes.hpp>

#include <unistd.h>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
UsbCoreViaSTM32F4::UsbCoreViaSTM32F4(USB_OTG_GlobalTypeDef * const p_usbCore, intptr_t p_usbPowerCtrl, const uint32_t p_rxFifoSzInWords)
  : m_usbCore(p_usbCore), m_usbPwrCtrl(reinterpret_cast<PowerAndClockGatingControl_t *>(p_usbPowerCtrl)), m_rxFifoSzInWords(p_rxFifoSzInWords) {
}

/*******************************************************************************
 *
 ******************************************************************************/
UsbCoreViaSTM32F4::~UsbCoreViaSTM32F4() {
}


/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::initialize() const {
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
UsbCoreViaSTM32F4::terminate() const {
    this->reset();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::registerDevice(UsbDeviceViaSTM32F4 &p_device) {
    assert(this->m_usbDevice == NULL);
    
    m_usbDevice = &p_device;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::unregisterDevice(UsbDeviceViaSTM32F4 &p_device) {
    assert(this->m_usbDevice == &p_device);
    
    this->m_usbDevice = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const {
    uint16_t offset;

    /* Host mode not (yet) supported */
    assert((this->m_usbCore->GUSBCFG & USB_OTG_GUSBCFG_FDMOD) != 0);

    if (p_endpoint == 0) {
        /* Calculate EP0 FIFO Offset based on Global Rx FIFO Length (which is configured in words) */
        this->m_usbCore->DIEPTXF0_HNPTXFSIZ = (p_fifoSzInWords << 16) | (((this->m_usbCore->GRXFSIZ) * sizeof(uint8_t)) & 0x0000FFFF);
    } else {
        if (p_endpoint == 1) {
            offset = (this->m_usbCore->DIEPTXF0_HNPTXFSIZ & 0x0000ffff) + (((this->m_usbCore->DIEPTXF0_HNPTXFSIZ >> 16) * sizeof(uint8_t)) & 0x0000FFFF);
        } else {
            offset = (this->m_usbCore->DIEPTXF[p_endpoint - 2] & 0x0000ffff) + (((this->m_usbCore->DIEPTXF[p_endpoint - 2] >> 16) * sizeof(uint8_t)) & 0x0000FFFF);
        }
        this->m_usbCore->DIEPTXF[p_endpoint - 1] = (p_fifoSzInWords << 16) | (offset & 0x0000FFFF);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::reset(void) const {
    this->performReset(USB_OTG_GRSTCTL_CSRST);

    usleep(25);

    this->disableInterrupt();
    this->stopPhy();

    USB_PRINTF("UsbCoreViaSTM32F4::%s(): Core Reset\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::performReset(const uint32_t p_reset) const {
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));

    this->m_usbCore->GRSTCTL |= p_reset;

    while (this->m_usbCore->GRSTCTL & p_reset);
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));

    // this->m_usbCore->GAHBCFG |= (USB_OTG_GAHBCFG_PTXFELVL | USB_OTG_GAHBCFG_TXFELVL);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setRxFifoSz(const uint16_t p_rxFifoSzInWords) const {
    // Minimum Rx FIFO size is 16, maximum size is 256 (all in words)
    assert((m_rxFifoSzInWords > 16) && (m_rxFifoSzInWords <= 256));

    this->m_usbCore->GRXFSIZ = p_rxFifoSzInWords;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::flushRxFifo(void) const {
    this->performReset(USB_OTG_GRSTCTL_RXFFLSH);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::flushTxFifo(const uint8_t p_fifo) const {
    assert(p_fifo <= 16);
    
    this->m_usbCore->GRSTCTL &= ~USB_OTG_GRSTCTL_TXFNUM;
    this->m_usbCore->GRSTCTL |= ((p_fifo << 6) & USB_OTG_GRSTCTL_TXFNUM);

    this->performReset(USB_OTG_GRSTCTL_TXFFLSH);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::enableInterrupt(void) const {
    this->m_usbCore->GINTSTS = 0xffffffff;
    this->m_usbCore->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::enableInterrupt(const Interrupt_t p_irq) const {
    this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
    this->m_usbCore->GINTMSK |= static_cast<uint32_t>(p_irq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::disableInterrupt(void) const {
    this->m_usbCore->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::disableInterrupt(const Interrupt_t p_irq) const {
    this->m_usbCore->GINTMSK &= ~(static_cast<uint32_t>(p_irq));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::startPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL &= ~USB_OTG_PCGCCTL_PHYSUSP;
    this->resumePhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::resumePhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);
    
    while (this->m_usbPwrCtrl->PCGCCTL  & USB_OTG_PCGCCTL_PHYSUSP);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::suspendPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL |= (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    while (!(this->m_usbPwrCtrl->PCGCCTL & USB_OTG_PCGCCTL_PHYSUSP));
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::stopPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL |= USB_OTG_PCGCCTL_PHYSUSP;
    this->suspendPhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::startTransceiver(void) const {
    this->m_usbCore->GCCFG |= USB_OTG_GCCFG_PWRDWN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::stopTransceiver(void) const {
    this->m_usbCore->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setupMode(const DeviceMode_e p_mode) {
    this->m_mode = p_mode;

    this->setupModeInHw(this->m_mode);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setupModeInHw(const DeviceMode_e p_mode) const {
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

        assert(0); // FIXME Host Mode not (yet?) supported
        break;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::start(void) const {
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
UsbCoreViaSTM32F4::stop(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s()\r\n", __func__);

    this->stopTransceiver();
    this->stopPhy();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleSessionEnd(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s()\r\n", __func__);

    if (this->m_mode == e_UsbDevice) {
        assert(this->m_usbDevice != NULL);
        
        this->m_usbDevice->stop();
    }

    /*
     * Re-enable / unmask the Session Request Interrupt here, otherwise we are not
     * notified if the device is plugged back in.
     */
    this->enableInterrupt(UsbCoreViaSTM32F4::e_SessionRequest);
}

/*******************************************************************************
 *
 ******************************************************************************/
/*
 * Table of OTG interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 */
const
UsbCoreViaSTM32F4::irq_handler_t UsbCoreViaSTM32F4::m_otgirq_handler[] = {
    { USB_OTG_GOTGINT_SEDET,    &UsbCoreViaSTM32F4::handleSessionEnd },
    { 0, NULL }
};

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleOtgIrq(void) const {
    uint32_t irqStatus = this->m_usbCore->GOTGINT;

    USB_PRINTF("UsbCoreViaSTM32F4::%s()\r\n", __func__);

    for (const UsbCoreViaSTM32F4::irq_handler_t *cur = UsbCoreViaSTM32F4::m_otgirq_handler; cur->m_irq != 0; cur++) {
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
UsbCoreViaSTM32F4::handleModeMismatchIrq(void) const {
    assert(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::wakeUpDetectedIrq(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s\r\n", __func__);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleSessionRequest(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s(): GINTMSK=%x\r\n", __func__, this->m_usbCore->GINTMSK);

    if (this->m_mode == DeviceMode_e::e_UsbDevice) {
        assert(this->m_usbDevice != NULL);
        
        this->m_usbDevice->start();
    }
}

/***************************************************************************//**
 * @brief USB Core Interrupt Handlers.
 * 
 * Table of interrupt handlers. Is handled in order from first to last, i.e.
 * functions listed earlier are handled before the functions listed later.
 * 
 * \see ::usb::stm32f4::UsbCoreViaSTM32F4::handleIrq
 ******************************************************************************/
const
UsbCoreViaSTM32F4::irq_handler_t UsbCoreViaSTM32F4::m_irq_handler[] = {
    { USB_OTG_GINTSTS_SRQINT,   &UsbCoreViaSTM32F4::handleSessionRequest },
    { USB_OTG_GINTSTS_MMIS,     &UsbCoreViaSTM32F4::handleModeMismatchIrq },
    { USB_OTG_GINTSTS_OTGINT,   &UsbCoreViaSTM32F4::handleOtgIrq },
    { USB_OTG_GINTSTS_WKUINT,   &UsbCoreViaSTM32F4::wakeUpDetectedIrq },
    { 0, NULL }
};

/***************************************************************************//**
 * @brief USB Core Interrupt Handler.
 * 
 * Handles the USB On-the-Go (OTG) Core's Interrupts as signalled in the
 * \c GINTSTS register. This handler will process the interrupts valid for both
 * Host- and Device-Mode.
 * 
 * If a valid USB Device Callback is registered in ::m_usbDevice, then further
 * processing of Device-specific interrupts is handled in the UsbDeviceViaSTM32F4
 * object.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleIrq(void) const {
    uint32_t irqStatus = this->m_usbCore->GINTSTS;
    uint32_t handledIrq = 0;

    this->disableInterrupt();

    USB_PRINTF("--> UsbCoreViaSTM32F4::%s(irqStatus=0x%x)\r\n", __func__, irqStatus);

    /* FIXME Make proper use of the Interrupt_t type and get rid of the static_cast's here. */
    for (const UsbCoreViaSTM32F4::irq_handler_t *cur = UsbCoreViaSTM32F4::m_irq_handler; cur->m_irq != 0; cur++) {
        if (irqStatus & cur->m_irq) {
            handledIrq |= cur->m_irq;
            (this->*(cur->m_fn))(); // Call member function via pointer
        }
    }
    
    if (this->m_usbDevice != NULL) {
        handledIrq |= this->m_usbDevice->handleIrq(irqStatus);
    }
    
    this->acknowledgeIrq(static_cast<typename UsbCoreViaSTM32F4::Interrupt_t>(handledIrq));

    USB_PRINTF("<-- UsbCoreViaSTM32F4::%s(): handled IRQ=%x (ignored=%x)\r\n\r\n", __func__, handledIrq, irqStatus & ~handledIrq);

    this->enableInterrupt();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::acknowledgeIrq(const Interrupt_t p_irq) const {
	this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setUsbTurnAroundTime(const uint8_t p_turnaroundTime) const {
	this->m_usbCore->GUSBCFG |= (p_turnaroundTime << USB_OTG_GUSBCFG_TRDT_Pos) & USB_OTG_GUSBCFG_TRDT_Msk;
}

/***************************************************************************//**
 * @brief Read receive Status from Rx FIFO.
 * 
 * The reads the USB Device's Rx FIFO Receive Status via the \c GRXSTSP register.
 * This means the status is read and removed (pop-ed) from the FIFO.
 ******************************************************************************/
uint32_t
UsbCoreViaSTM32F4::getRxStatus(void) const {
	return this->m_usbCore->GRXSTSP;
}
/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09 */
  
