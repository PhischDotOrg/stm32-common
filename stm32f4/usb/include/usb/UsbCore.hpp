/*-
 * $Copyright$
-*/

#ifndef _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70
#define	_USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70

#include <stm32f4/RccViaSTM32F4.hpp>
#include <gpio/GpioPin.hpp>

#include <stm32f4xx.h>

namespace devices {
    class ScbViaSTM32F4;
    template<typename ScbT = devices::ScbViaSTM32F4> class NvicViaSTM32F4T;
}

namespace usb {
    namespace stm32f4 {

template<typename UsbCoreT> class UsbDeviceViaUsbCoreT;

/*******************************************************************************
 *
 ******************************************************************************/
class UsbCore {
public:
            UsbCore(USB_OTG_GlobalTypeDef * const p_usbCore, intptr_t p_usbPowerCtrl, const uint32_t p_rxFifoSzInWords);
    virtual ~UsbCore();

    void    registerDevice(UsbDeviceViaUsbCoreT<UsbCore> &p_device);
    void    unregisterDevice(UsbDeviceViaUsbCoreT<UsbCore> &p_device);

    void    setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const;
    void    flushTxFifo(const uint8_t p_fifo) const;

    void    reset(void) const;
    
    enum DeviceMode_e {
        e_UsbDevice,
        e_UsbHost
    };

    void    setupMode(const DeviceMode_e p_mode);

    intptr_t getBaseAddr(void) const {
        return reinterpret_cast<intptr_t>(this->m_usbCore);
    };

    void    start(void) const;
    void    stop(void) const;

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
            e_Disconnected          = USB_OTG_GINTMSK_DISCINT,
            e_SessionRequest        = USB_OTG_GINTMSK_SRQIM,
            e_WakeUpDetected        = USB_OTG_GINTMSK_WUIM,
    } Interrupt_t;

    void    disableInterrupt(const Interrupt_t p_irq) const;
    void    enableInterrupt(const Interrupt_t p_irq) const;

    void    suspendPhy(void) const;
    void    resumePhy(void) const;
    
    void    handleIrq(void) const;

    void    acknowledgeIrq(const Interrupt_t p_irq) const;

    uint32_t    getRxStatus(void) const;

    void    setUsbTurnAroundTime(const uint8_t p_turnaroundTime) const;

protected:
    void initialize(void) const;
    void terminate(void) const;
    
private:
    typedef struct PowerAndClockGatingControl_s {
        volatile uint32_t PCGCCTL;
    } PowerAndClockGatingControl_t;
    
    USB_OTG_GlobalTypeDef * const           m_usbCore;
    PowerAndClockGatingControl_t * const    m_usbPwrCtrl;
    const uint32_t                          m_rxFifoSzInWords;
    UsbDeviceViaUsbCoreT<UsbCore> *         m_usbDevice;
    DeviceMode_e                            m_mode;

    typedef void (usb::stm32f4::UsbCore::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        uint32_t            m_irq;
        irq_handler_fn      m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];
    static const irq_handler_t m_otgirq_handler[];

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
};


/*******************************************************************************
 *
 ******************************************************************************/
template<gpio::GpioAccessViaSTM32F4::Function_e GpioFunctionT>
class UsbCoreGpioHelperT {
public:
    static void initalize(gpio::GpioPin &p_pinDm, gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId) {
        p_pinDm.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
        p_pinDp.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
        p_pinVbus.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
        p_pinId.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
    }
    
    static void terminate(gpio::GpioPin &p_pinDm, gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId) {
        p_pinDm.disable();
        p_pinDp.disable();
        p_pinVbus.disable();
        p_pinId.disable();
    }
};

/*******************************************************************************
 * Forward declare and typedef the instance specific types for the USB cores
 ******************************************************************************/
template<intptr_t UsbT, typename GpioPinT = gpio::GpioPin, typename RccT = devices::RccViaSTM32F4, typename NvicT = devices::NvicViaSTM32F4T<> > class UsbCoreFromAddressPointerT;

#if defined(USB_OTG_FS_PERIPH_BASE)
typedef UsbCoreFromAddressPointerT<USB_OTG_FS_PERIPH_BASE>   UsbFullSpeedCore;
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
typedef UsbCoreFromAddressPointerT<USB_OTG_HS_PERIPH_BASE>   UsbHighSpeedCore;
#endif

/*******************************************************************************
 * Helper Template to resolve the SPI Address to the corresponding GPIO
 * Alternate Function Defintion
 ******************************************************************************/
template<intptr_t> struct UsbCoreGpioFunction;

#if defined(USB_OTG_FS_PERIPH_BASE)
template<> struct UsbCoreGpioFunction<USB_OTG_FS_PERIPH_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_UsbFs;
};
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
template<> struct UsbCoreGpioFunction<USB_OTG_HS_PERIPH_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_UsbHs;
};
#endif

/*******************************************************************************
 * Helper Template to resolve the USB Core Base Address to the corresponding
 * RCC Function Defintion
 ******************************************************************************/
template<intptr_t> struct UsbCoreRccFunction;

#if defined(USB_OTG_FS_PERIPH_BASE)
template<> struct UsbCoreRccFunction<USB_OTG_FS_PERIPH_BASE> {
    static const devices::RccViaSTM32F4::FunctionAHB2_t m_type = devices::RccViaSTM32F4::e_OtgFs;
};
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
template<> struct UsbCoreRccFunction<USB_OTG_HS_PERIPH_BASE> {
    static const devices::RccViaSTM32F4::FunctionAHB1_t m_type = devices::RccViaSTM32F4::e_OtgHs;
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t UsbT, typename GpioPinT /* = gpio::GpioPin */, typename RccT /* = devices::RccViaSTM32F4 */, typename NvicT /* = devices::NvicViaSTM32F4*/>
class UsbCoreFromAddressPointerT : public UsbCore {
private:
    NvicT &     m_nvic;
    RccT &      m_rcc;
    GpioPinT &  m_pinDm;
    GpioPinT &  m_pinDp;
    GpioPinT &  m_pinVbus;
    GpioPinT &  m_pinId;
    
public:
    UsbCoreFromAddressPointerT(NvicT &p_nvic, RccT &p_rcc, GpioPinT &p_pinDm, GpioPinT &p_pinDp, GpioPinT &p_pinVbus, GpioPinT &p_pinId, uint32_t p_rxFifoSzInWords = 128)
      : UsbCore(reinterpret_cast<USB_OTG_GlobalTypeDef *>(UsbT), UsbT + USB_OTG_PCGCCTL_BASE, p_rxFifoSzInWords), m_nvic(p_nvic), m_rcc(p_rcc), m_pinDm(p_pinDm), m_pinDp(p_pinDp), m_pinVbus(p_pinVbus), m_pinId(p_pinId) {
        m_rcc.enable(UsbCoreRccFunction<UsbT>::m_type);
        UsbCoreGpioHelperT< UsbCoreGpioFunction<UsbT>::m_type >::initalize(m_pinDm, m_pinDp, m_pinVbus, m_pinId);
        this->initialize();
        m_nvic.enableIrq(*this);
    }
    
    virtual ~UsbCoreFromAddressPointerT() {
        m_nvic.disableIrq(*this);
        this->terminate();
        UsbCoreGpioHelperT< UsbCoreGpioFunction<UsbT>::m_type >::terminate(m_pinDm, m_pinDp, m_pinVbus, m_pinId);
        m_rcc.disable(UsbCoreRccFunction<UsbT>::m_type);
    }

#if 0    
    void suspendFromIrq(void) const {
        
    }
#endif
};

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif	/* _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70 */
