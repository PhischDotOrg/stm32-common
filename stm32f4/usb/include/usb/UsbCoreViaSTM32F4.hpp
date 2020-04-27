/*-
 * $Copyright$
-*/

#ifndef _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70
#define	_USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70

#include <stm32f4/RccViaSTM32F4.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32f4xx.h>

namespace usb {
    namespace stm32f4 {

        class UsbDeviceViaSTM32F4;

/*******************************************************************************
 *
 ******************************************************************************/
class UsbCoreViaSTM32F4 {
public:
/*******************************************************************************
 * Public Type definitions
 ******************************************************************************/
    enum DeviceMode_e {
        e_UsbDevice,
        e_UsbHost
    };

    typedef enum Interrupt_e {
        e_ModeMismatch          = USB_OTG_GINTMSK_MMISM,
        e_OnTheGo               = USB_OTG_GINTMSK_OTGINT,
        e_StartOfFrame          = USB_OTG_GINTMSK_SOFM,
        e_RxFifoNonEmpty        = USB_OTG_GINTMSK_RXFLVLM,
        e_TxFifoEmpty           = USB_OTG_GINTMSK_NPTXFEM,
        e_InNackEffective       = USB_OTG_GINTMSK_GINAKEFFM,
        e_OutNackEffective      = USB_OTG_GINTMSK_GONAKEFFM,
        e_EarlySuspend          = USB_OTG_GINTMSK_ESUSPM,
        e_Suspend               = USB_OTG_GINTMSK_USBSUSPM,
        e_Reset                 = USB_OTG_GINTMSK_USBRST,
        e_EnumerationDone       = USB_OTG_GINTMSK_ENUMDNEM,
        e_IsoOutDropped         = USB_OTG_GINTMSK_ISOODRPM,
        e_EndOfPeriodicFrame    = USB_OTG_GINTMSK_EOPFM,
        e_EndpointMismatch      = USB_OTG_GINTMSK_EPMISM,
        e_InEndpoint            = USB_OTG_GINTMSK_IEPINT,
        e_OutEndpoint           = USB_OTG_GINTMSK_OEPINT,
        e_IncomplIsoIn          = USB_OTG_GINTMSK_IISOIXFRM,
        e_IncomplPeriodic       = USB_OTG_GINTMSK_PXFRM_IISOOXFRM,
        e_FetchSuspended        = USB_OTG_GINTMSK_FSUSPM,
        e_HostPort              = USB_OTG_GINTMSK_PRTIM, /* Host Only */
        e_HostChannel           = USB_OTG_GINTMSK_HCIM, /* Host Only */
        e_PeriodicTxEmpty       = USB_OTG_GINTMSK_PTXFEM,
        e_ConnectorIdStatus     = USB_OTG_GINTMSK_CIDSCHGM,
        e_Disconnected          = USB_OTG_GINTMSK_DISCINT, /* IRQ Mask bit is for both Device and Host Mode, but IRQ Status is Host Only. */
        e_SessionRequest        = USB_OTG_GINTMSK_SRQIM,
        e_WakeUpDetected        = USB_OTG_GINTMSK_WUIM,
    } Interrupt_t;

/*******************************************************************************
 * Public Methods
 ******************************************************************************/
public:
            UsbCoreViaSTM32F4(USB_OTG_GlobalTypeDef * const p_usbCore, intptr_t p_usbPowerCtrl, const uint32_t p_rxFifoSzInWords);
    virtual ~UsbCoreViaSTM32F4();

/*******************************************************************************
 * Interface Core <-> Device
 ******************************************************************************/
    void    registerDevice(UsbDeviceViaSTM32F4 &p_device);
    void    unregisterDevice(UsbDeviceViaSTM32F4 &p_device);

    void    setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const;
    void    flushTxFifo(const uint8_t p_fifo) const;

    void    reset(void) const;
    
    void    setupMode(const DeviceMode_e p_mode);

    intptr_t getBaseAddr(void) const {
        return reinterpret_cast<intptr_t>(this->m_usbCore);
    };

    void    start(void) const;
    void    stop(void) const;

    void    disableInterrupt(const Interrupt_t p_irq) const;
    void    enableInterrupt(const Interrupt_t p_irq) const;

    void    suspendPhy(void) const;
    void    resumePhy(void) const;
    
    void    handleIrq(void) const;

    void    acknowledgeIrq(const Interrupt_t p_irq) const;

    uint32_t    getRxStatus(void) const;

    void    setUsbTurnAroundTime(const uint8_t p_turnaroundTime) const;

/*******************************************************************************
 * Protected Methods
 ******************************************************************************/
protected:
    void initialize(void) const;
    void terminate(void) const;
    
/*******************************************************************************
 * Private Types
 ******************************************************************************/
private:
    typedef struct PowerAndClockGatingControl_s {
        volatile uint32_t PCGCCTL;
    } PowerAndClockGatingControl_t;
    
    typedef void (usb::stm32f4::UsbCoreViaSTM32F4::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        uint32_t            m_irq;
        irq_handler_fn      m_fn;
    } irq_handler_t;

/*******************************************************************************
 * Private Attributes
 ******************************************************************************/
private:
    USB_OTG_GlobalTypeDef * const           m_usbCore;
    PowerAndClockGatingControl_t * const    m_usbPwrCtrl;
    const uint32_t                          m_rxFifoSzInWords;
    UsbDeviceViaSTM32F4 *                   m_usbDevice;
    DeviceMode_e                            m_mode;

    static const irq_handler_t m_irq_handler[];
    static const irq_handler_t m_otgirq_handler[];

/*******************************************************************************
 * Private Methods
 ******************************************************************************/
private:
    void    disableInterrupt(void) const;
    void    enableInterrupt(void) const;

    void    stopPhy(void) const;
    void    startPhy(void) const;

    void    startTransceiver(void) const;
    void    stopTransceiver(void) const;

    void    setRxFifoSz(const uint16_t p_rxFifoSzInWords) const;

    void    flushRxFifo(void) const;
    
    void    performReset(const uint32_t p_reset) const;
    
    void    setupModeInHw(const DeviceMode_e p_mode) const;

    void    handleModeMismatchIrq(void) const;
    void    handleOtgIrq(void) const;
    void    handleSessionRequest(void) const;
    void    handleSessionEnd(void) const;
    void    wakeUpDetectedIrq(void) const;
};

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif	/* _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70 */
