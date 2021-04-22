/*-
 * $Copyright$
-*/

#ifndef _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09
#define	_USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09

#include <usb/UsbCoreViaSTM32F4.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <usb/UsbTypes.hpp>

#include <cassert>
#include <cstdlib>

extern "C" {
    #if (__cplusplus == 201103L) || (__cplusplus == 201402L) || (__cplusplus == 201703L) || (__cplusplus > 201703L)
        int usleep(unsigned p_usec);
    #else
        #include <unistd.h>
    #endif
}

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/***************************************************************************//**
 * @brief Initialize the USB Core.
 *
 * This method will intialize the Hardware by calling #reset internally.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::initialize() const {
    this->reset();
}

/***************************************************************************//**
 * @brief Terminate the USB Core.
 *
 * This method will reset the Hardware by calling #reset internally.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::terminate() const {
    this->reset();
}

/***************************************************************************//**
 * @brief Set up the Tx FIFO for a given endpoint.
 *
 * Sets up the Tx FIFO for a given endpoint.
 *
 * \warning Please note that there is currently no check on the FIFO sizes of all
 * Endpoints. I.e. the FIFO sizes can be requested such that the total amount of
 * all enpoint FIFOs exceeds the hardware capabilities.
 *
 * @param p_endpoint IN Endpoint Number whose FIFO is to be set up.
 * @param p_fifoSzInWords Endpoint's Tx FIFO Size in Words.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const {
    uint16_t offset;

     /*
      * Size in GRXFSIZ and DIEPTXFx registers are in units of words, but it seems that offset
      * needs to be given in units of bytes.
      */
    unsigned offsetFactor = 1; // sizeof(uint32_t);

    /* FIXME It would be nice if the FIFO Size could be checked; ideally at compile time. */

    /* TODO Host mode not (yet) supported */
    assert((this->m_usbCore->GUSBCFG & USB_OTG_GUSBCFG_FDMOD) != 0);

    if (p_endpoint == 0) {
        /* Calculate EP0 FIFO Offset based on Global Rx FIFO Length (which is configured in words) */
        this->m_usbCore->DIEPTXF0_HNPTXFSIZ = (p_fifoSzInWords << 16) | (((this->m_usbCore->GRXFSIZ) * offsetFactor) & 0x0000FFFF);
    } else {
        if (p_endpoint == 1) {
            offset = (this->m_usbCore->DIEPTXF0_HNPTXFSIZ & 0x0000ffff) + (((this->m_usbCore->DIEPTXF0_HNPTXFSIZ >> 16) * offsetFactor) & 0x0000FFFF);
        } else {
            offset = (this->m_usbCore->DIEPTXF[p_endpoint - 2] & 0x0000ffff) + (((this->m_usbCore->DIEPTXF[p_endpoint - 2] >> 16) * offsetFactor) & 0x0000FFFF);
        }
        this->m_usbCore->DIEPTXF[p_endpoint - 1] = (p_fifoSzInWords << 16) | (offset & 0x0000FFFF);
    }
}

/***************************************************************************//**
 * @brief Resets the USB Core.
 *
 * Performs a reset of the USB Core's Hardware.
 *
 * Internally, this method uses #performReset with the \c CSRST flag to reset
 * the Device Hardware.
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::reset(void) const {
    this->performReset(USB_OTG_GRSTCTL_CSRST);

    this->disableInterrupt();
    this->suspendPhy();

    USB_PRINTF("UsbCoreViaSTM32F4::%s(): Core Reset\r\n", __func__);
}

/***************************************************************************//**
 * @brief Performs a partial reset of the USB Hardware.
 *
 * Performs a partial reset of the USB Hardware via the \c GRSTCTL register.
 *
 * @param p_reset Type of reset to be performed. Must be one of the allowed reset
 *   types as defined in the hardware documentation of the \c GRSTCTL register.
 *
 * @attention This method blocks until the reset is complete by waiting for the
 *   \c AHBIDL bit in the \c GRSTCTL register to be set.
 *
 * @bug This method should probably use an enum to define the allowed types of
 *   reset.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::performReset(const uint32_t p_reset) const {
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));

    /* FIXME Use an enum to define the allowed types of resets */
    this->m_usbCore->GRSTCTL |= p_reset;

#if !defined(HOSTBUILD)
    while (this->m_usbCore->GRSTCTL & p_reset);
    while (!(this->m_usbCore->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL));
#endif

    // this->m_usbCore->GAHBCFG |= (USB_OTG_GAHBCFG_PTXFELVL | USB_OTG_GAHBCFG_TXFELVL);
}

/***************************************************************************//**
 * @brief Flushes the Rx FIFO.
 *
 * Flushes the Rx FIFO by performing the \c RXFFLSH Reset via #performReset.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::flushRxFifo(void) const {
    this->performReset(USB_OTG_GRSTCTL_RXFFLSH);
}

/***************************************************************************//**
 * @brief Flushes a given Tx FIFO.
 *
 * @param p_fifo Index of Tx FIFO to be flushed.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::flushTxFifo(const uint8_t p_fifo) const {
    assert(p_fifo <= 16);

    this->m_usbCore->GRSTCTL &= ~USB_OTG_GRSTCTL_TXFNUM;
    this->m_usbCore->GRSTCTL |= ((p_fifo << 6) & USB_OTG_GRSTCTL_TXFNUM);

    this->performReset(USB_OTG_GRSTCTL_TXFFLSH);
}

/***************************************************************************//**
 * @brief Resumes the USB PHY.
 *
 * Resumes the USB PHY by enabling the clocks via the \c PCGCCTL register.
 *
 * \attention This method blocks until the Hardware reports the PHY as actually
 * running via the \c PCGCCTL register.
 *
 * \bug I think the \c STPPCLK and the \c GATEHCLK should be separated as only
 * the \c STPPCLK bit should be set when a _USB Suspend_ IRQ ( #e_Suspend )
 * occurs.
 *
 * \see #suspendPhy
 ******************************************************************************/
void
UsbCoreViaSTM32F4::resumePhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL &= ~(USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    while (this->m_usbPwrCtrl->PCGCCTL  & USB_OTG_PCGCCTL_PHYSUSP);
}

/***************************************************************************//**
 * @brief Suspends the USB PHY.
 *
 * Suspends the USB PHY by disabling the clocks via the \c PCGCCTL register.
 * - Stops the PHY 48 MHz Clock (\c STPPCLK Bit). This will shut off most
 *   hardware logic except the resume / remote wake-up capabilities.
 * - Gates the HCLK (\c GATEHCLK Bit). This will shut down all hardware logic
 *   except the register read / write interface.
 *
 * \attention This method blocks until the Hardware reports the PHY as actually
 * suspended via the \c PCGCCTL register.
 *
 * \bug I think the \c STPPCLK and the \c GATEHCLK should be separated as only
 * the \c STPPCLK bit should be set when a _USB Suspend_ IRQ ( #e_Suspend )
 * occurs.
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::suspendPhy(void) const {
    this->m_usbPwrCtrl->PCGCCTL |= (USB_OTG_PCGCCTL_STOPCLK | USB_OTG_PCGCCTL_GATECLK);

    while ((this->m_usbPwrCtrl->PCGCCTL & USB_OTG_PCGCCTL_PHYSUSP) != USB_OTG_PCGCCTL_PHYSUSP);
}

/***************************************************************************//**
 * @brief Set up the USB Mode in the Hardware.
 *
 * This method will set up the requested USB Mode (Host or Device) in the
 * USB Core Hardware.
 *
 * In case of Device Mode, the method will
 * - Enable Vbus sensing by setting the \c VBUSBSEN Bit in the \c GCCFG register.
 * - Force Device Mode by setting the \c FDMOD bit in the \c GUSBCFG register.
 * - Start the PHY via #startPhy.
 * - Start the Transceiver via #startTransceiver.
 * - Flush the Rx FIFO via #flushRxFifo
 * - Flush all Tx FIFOs via #flushTxFifo
 * - Set up the Rx FIFO Size via #setRxFifoSz
 *
 * @bug USB Host Mode (#e_UsbHost) is not yet supported.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::setupModeInHw(const DeviceMode_e p_mode) const {
    switch (p_mode) {
    case e_UsbDevice:
        this->m_usbCore->GCCFG |= (USB_OTG_GCCFG_VBUSBSEN | USB_OTG_GCCFG_NOVBUSSENS) ;
        this->m_usbCore->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;

        this->m_usbCore->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL_Msk;

        this->resumePhy();
        this->startTransceiver();

        /*
         * The USB Turn-around Time and the FS should be set up according
         * to the data sheet description of the register, but I found that
         * it doesn't make a difference in my tests.
         */
        this->m_usbCore->GUSBCFG |= ((0x6 << USB_OTG_GUSBCFG_TRDT_Pos) & USB_OTG_GUSBCFG_TRDT_Msk);
        // this->m_usbCore->GUSBCFG |= (17 << USB_OTG_GUSBCFG_TOCAL_Pos) & USB_OTG_GUSBCFG_TOCAL_Msk;

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

/***************************************************************************//**
 * @brief Starts the USB Core.
 *
 * Starts the USB Core Operation by:
 *
 * - Starting the PHY.
 * - Starting the USB Transceiver.
 * - Enabling the IRQ.
 *
 * \see #stop
 ******************************************************************************/
void
UsbCoreViaSTM32F4::start(void) const {
    this->startTransceiver();
    this->resumePhy();

    this->enableInterrupt();
}

/***************************************************************************//**
 * @brief Stops the USB Core.
 *
 * Stops the USB Core by
 * - Stopping the Transceiver.
 * - Stopping the PHY.
 *
 * \see #start
 ******************************************************************************/
void
UsbCoreViaSTM32F4::stop(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s()\r\n", __func__);

    /* TODO start() enables the IRQs, should we disable them here? */

    this->suspendPhy();
    this->stopTransceiver();
}

/***************************************************************************//**
 * @brief Handle the _Session End_ IRQ
 *
 * In USB Device Mode, this interrupt is triggered when the host is disconnected,
 * e.g. when the USB cable is unplugged.
 *
 * \attention This will re-enable IRQs and unmask the _Session Request_ IRQ. This
 * is needed so the Application can react if the device is plugged back in.
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

/***************************************************************************//**
 * @brief USB Core On-the-Go (OTG) Interrupt Handlers.
 *
 * Table of On-the-Go (OTG) interrupt handlers. Is handled in order from first
 * to last, i.e. functions listed earlier are handled before the functions
 * listed later.
 *
 * \see #handleOtgIrq
 ******************************************************************************/
const
UsbCoreViaSTM32F4::irq_handler_t UsbCoreViaSTM32F4::m_otgirq_handler[] = {
    { USB_OTG_GOTGINT_SEDET,    &UsbCoreViaSTM32F4::handleSessionEnd },
    { 0, NULL }
};

/***************************************************************************//**
 * @brief USB Core On-the-Go (OTG) Interrupt Handler.
 *
 * Handles the USB On-the-Go (OTG) Core's Interrupts as signalled in the
 * \c GOTGINT register.
 *
 * \see #m_otgirq_handler
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

/***************************************************************************//**
 * @brief Handle the USB Mode Mismatch IRQ.
 *
 * According to the Data Sheet, this interrupt is triggered when the Software
 * attempts to access a Host-mode register when operating in Device-Mode and
 * vice-versa.
 *
 * In Debug Mode, this method will \c assert() and thus serves as a debugging aid.
 *
 * \see #e_ModeMismatch
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleModeMismatchIrq(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s\r\n", __func__);
    // assert(false);
}

/***************************************************************************//**
 * @brief Handle the _Wake-up Detected_ IRQ.
 *
 * \bug From the Datasheed it seems this is the complement to the _USB Suspend_
 * IRQ (#e_Suspend). We should probably call #resumePhy here.
 ******************************************************************************/
void
UsbCoreViaSTM32F4::wakeUpDetectedIrq(void) const {
    USB_PRINTF("UsbCoreViaSTM32F4::%s\r\n", __func__);
    /* FIXME Call resumePhy here? */
}

/***************************************************************************//**
 * @brief Handle the _Session Request_ IRQ.
 *
 * In USB Device Mode, this is triggered when the Device is connected to a USB
 * Host.
 *
 * \see #m_irq_handler
 * \see #e_SessionRequest
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
 * \see #handleIrq
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
 * If a valid USB Device Callback is registered in #m_usbDevice, then further
 * processing of Device-specific interrupts is handled in the UsbDeviceViaSTM32F4
 * object.
 *
 * \see #m_irq_handler
 *
 ******************************************************************************/
void
UsbCoreViaSTM32F4::handleIrq(void) const {
    uint32_t irqStatus = this->m_usbCore->GINTSTS;
    uint32_t handledIrq = 0;

    this->disableInterrupt();

    USB_PRINTF("--> UsbCoreViaSTM32F4::%s(irqStatus=0x%x)\r\n", __func__, irqStatus);

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

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _USBCORE_CPP_04381907_921b_416c_8108_0ffc6945ff09 */

