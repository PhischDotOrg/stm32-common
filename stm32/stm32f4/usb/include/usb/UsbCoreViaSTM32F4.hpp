/*-
 * $Copyright$
-*/

#ifndef _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70
#define	_USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70

#include <stm32/Engine.hpp>

#include <stm32f4xx.h>

#include <cassert>

/*******************************************************************************
 * This file is needed / used from <stm32f4/NvicViaSTM32F4.hpp -- in order to
 * break up a chicken-and-egg problem between the types, we need a forward
 * declaration here.
 ******************************************************************************/
namespace devices {
    class ScbViaSTM32F4;
    template<typename ScbT = devices::ScbViaSTM32F4> class NvicViaSTM32F4T;
}

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

    class UsbDeviceViaSTM32F4;

/***************************************************************************//**
 * \brief Driver for the STM32F4 USB Core.
 * 
 * Class that encapsulates the operation of the STM32F4's USB On-the-Go (OTG)
 * Core. This is code common to both Device and Host mode operation.
 * 
 * \bug Currently, the #UsbCoreViaSTM32F4 class only supports USB Device Mode.
 ******************************************************************************/
class UsbCoreViaSTM32F4 {
    friend class UsbCoreViaSTM32F4Test;
public:
/*******************************************************************************
 * Public Type definitions
 ******************************************************************************/

    /**
     * @brief USB Hardware Mode (Host vs. Device).
     * 
     * \see #setupMode
     */
    enum DeviceMode_e {
        /** @brief USB Device Mode. */
        e_UsbDevice,
        /** @brief USB Host Mode. */
        e_UsbHost
    };

    /**
     * @brief STM32F4 USB Core Interrupts.
     * 
     * This enum defines the possible STM32F4 USB Core Interrupts as described by the
     * Datasheet for the \c GINTMSK as well as for the \c GINTSTS Registers.
     * 
     * \see #m_irq_handler
     * \see #handleIrq
     */
    typedef enum Interrupt_e : uint32_t {
        e_None                  = 0,
        /**
         * @brief USB Mode Mismatch Interrupt.
         * 
         * \see #handleModeMismatchIrq
         */
        e_ModeMismatch          = USB_OTG_GINTMSK_MMISM,
        e_OnTheGo               = USB_OTG_GINTMSK_OTGINT,
        e_StartOfFrame          = USB_OTG_GINTMSK_SOFM,
        e_RxFifoNonEmpty        = USB_OTG_GINTMSK_RXFLVLM,
        e_TxFifoEmpty           = USB_OTG_GINTMSK_NPTXFEM,
        e_InNackEffective       = USB_OTG_GINTMSK_GINAKEFFM,
        e_OutNackEffective      = USB_OTG_GINTMSK_GONAKEFFM,
        e_EarlySuspend          = USB_OTG_GINTMSK_ESUSPM,
        /**
         * @brief USB Suspend IRQ.
         * 
         * \see ::usb::stm32f4::UsbDeviceViaSTM32F4::handleUsbSuspendIrq
         * 
         */
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
        /**
         * @brief Session Request IRQ.
         * 
         * \see #handleSessionRequest
         */
        e_SessionRequest        = USB_OTG_GINTMSK_SRQIM,
        e_WakeUpDetected        = USB_OTG_GINTMSK_WUIM,
    } Interrupt_t;

/*******************************************************************************
 * Public Methods
 ******************************************************************************/
public:
    /**
     * @brief Construct a new UsbCoreViaSTM32F4 object.
     * 
     * This constructor is used from the helper sub-class sets up the references
     * to the Hardware addresses based on the desired USB Mode (High Speed vs. Full Speed).
     * 
     * @see UsbCoreViaSTM32F4FromAddressPointerT
     * 
     * @param p_usbCore Pointer to USB Core's Registers.
     * @param p_usbPowerCtrl Pointer to USB Core's \c PCGCCTL Register.
     * @param p_rxFifoSzInWords USB Rx FIFO Size in Words.
     */
    UsbCoreViaSTM32F4(USB_OTG_GlobalTypeDef * const p_usbCore, intptr_t p_usbPowerCtrl, const uint32_t p_rxFifoSzInWords)
      : m_usbCore(p_usbCore), m_usbPwrCtrl(reinterpret_cast<PowerAndClockGatingControl_t *>(p_usbPowerCtrl)), m_rxFifoSzInWords(p_rxFifoSzInWords), m_usbDevice(nullptr), m_mode(DeviceMode_e::e_UsbDevice) {
        // Minimum Rx FIFO size is 16, maximum size is 256 (all in words)
        assert((m_rxFifoSzInWords >= 16) && (m_rxFifoSzInWords <= 256));
    }

    /**
     * @brief Destroy the UsbCoreViaSTM32F4 object.
     * 
     * Virtual because the class is intended to be sub-classed.
     *
     * @see UsbCoreViaSTM32F4FromAddressPointerT
     */
    virtual ~UsbCoreViaSTM32F4() {

    }

/*******************************************************************************
 * Interface Core <-> Device
 ******************************************************************************/
    /**
     * @brief Register USB Device Mode specific Callback Object.
     * 
     * This method is only allowed to be called if no previous object has been
     * registered before.
     *
     * While this would not be required, the pattern is used to discourage calls
     * that would not be needed otherwise.
     *
     * @param p_device UsbDeviceViaSTM32F4 object to be used as a callback.
     *
     * \see unregisterDevice
     */
    void registerDevice(const UsbDeviceViaSTM32F4 &p_device) {
        assert(this->m_usbDevice == nullptr);
        m_usbDevice = &p_device;
    }

    /**
     * @brief Unregister USB Device Mode specific Callback Object.
     * 
     * This method is only allowed to be called if if an object has been
     * registered before.
     *
     * While this would not be required, the pattern is used to discourage calls
     * that would not be needed otherwise.
     * 
     * \see registerDevice
     */
    void unregisterDevice(void) {
        assert(this->m_usbDevice != nullptr);
        this->m_usbDevice = nullptr;
    }

    void    setupTxFifo(const unsigned p_endpoint, const uint16_t p_fifoSzInWords) const;
    void    flushTxFifo(const uint8_t p_fifo) const;

    void    reset(void) const;

    /**
     * @brief Set up the USB Mode (Host vs. Device).
     * 
     * Used by the Mode-dependent class to set up the USB Mode in the STM32F4 Hardware.
     * 
     * @param p_mode USB MOde (Host vs. Device).
     * 
     * \see UsbDeviceViaSTM32F4
     * \see setupModeInHw
     */
    void setupMode(const DeviceMode_e p_mode) {
        this->m_mode = p_mode;
        this->setupModeInHw(this->m_mode);
    }

    /**
     * @brief Get the Base Address of the USB Core's Registers.
     * 
     * Used by the Mode-dependent class to figure out the Base Address of the Mode-specific registers.
     * 
     * \see UsbDeviceViaSTM32F4
     *
     * @return constexpr intptr_t Base Address of the USB Core Registers.
     * 
     */
    intptr_t getBaseAddr(void) const {
        return reinterpret_cast<intptr_t>(this->m_usbCore);
    };

    void    start(void) const;
    void    stop(void) const;

    /**
     * @brief Disables a given Interrupt.
     * 
     * Masks the given Interrupt in the \c GINTMSK register.
     * 
     * @param p_irq Interrupt to be disabled.
     */
    void    disableInterrupt(const Interrupt_t p_irq) const {
        this->m_usbCore->GINTMSK &= ~(static_cast<uint32_t>(p_irq));
    }

    /**
     * @brief Enables a given Interrupt.
     * 
     * Unmasks the given Interrupt in the \c GINTMSK register.
     *
     * @param p_irq Interrupt to be enabled.
     */
    void    enableInterrupt(const Interrupt_t p_irq) const {
        this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
        this->m_usbCore->GINTMSK |= static_cast<uint32_t>(p_irq);
    }

    void    suspendPhy(void) const;
    void    resumePhy(void) const;
    
    void    handleIrq(void) const;

    /**
     * @brief Acknowledge a given IRQ on the USB Core.
     * 
     * @param p_irq IRQ that is to be acknowledged.
     */
    void acknowledgeIrq(const Interrupt_t p_irq) const {
	    this->m_usbCore->GINTSTS = static_cast<uint32_t>(p_irq);
    }

    /***************************************************************************//**
    * @brief Read receive Status from Rx FIFO.
    * 
    * The reads the USB Device's Rx FIFO Receive Status via the \c GRXSTSP register.
    * This means the status is read and removed (pop-ed) from the FIFO.
    ******************************************************************************/
    uint32_t getRxStatus(void) const {
	    return this->m_usbCore->GRXSTSP;
    }

    /**
     * @brief Set the USB Turn Around Time.
     * 
     * Sets the USB Turnsaround time \c TRDT in Register \c GUSBCFG.
     * 
     * @param p_turnaroundTime Turnaround time in PHY Clocks.
     */
    void setUsbTurnAroundTime(const uint8_t p_turnaroundTime) const {
    	this->m_usbCore->GUSBCFG |= (p_turnaroundTime << USB_OTG_GUSBCFG_TRDT_Pos) & USB_OTG_GUSBCFG_TRDT_Msk;
    }

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
    /**
     * @brief Type definition for access to the \c PCGCCTL register.
     * 
     * Because the register doesn't seem to be declared in the ST-provided C-style
     * header files, we're setting up our own type definition here.
     */
    typedef struct PowerAndClockGatingControl_s {
        /** @brief Power and Clock Gating Control Register.
         * 
         * See the STM32F4 Device Documentation for details.
         */
        volatile uint32_t PCGCCTL;
    } PowerAndClockGatingControl_t;
    
    /**
     * @brief Type declaration for IRQ Handlers.
     * 
     * IRQ handlers are Pointers to Member Functions. This typedef is declared
     * for convenience when using the pointers.
     */
    typedef void (stm32::usb::UsbCoreViaSTM32F4::*irq_handler_fn)() const;

    /**
     * @brief Type definition for the USB Core IRQ handler table.
     * 
     * Type definition for the various IRQ handler tables used in #UsbCoreViaSTM32F4.
     * 
     * \see m_irq_handler
     * \see m_otgirq_handler
     */
    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        uint32_t            m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

/*******************************************************************************
 * Private Attributes
 ******************************************************************************/
private:
    /** @brief Reference to the USB Core's Registers. */
    USB_OTG_GlobalTypeDef * const           m_usbCore;

    /**
     * @brief Reference to the USB Core's \c PCGCCTL Register.
     * 
     * This register allows control of the USB Clocks, most importantly the PHY.
     * 
     * \see #startPhy
     * \see #stopPhy
     * \see #suspendPhy
     * \see #resumePhy
     */
    PowerAndClockGatingControl_t * const    m_usbPwrCtrl;

    /**
     * @brief USB Rx FIFO Size in Words.
     * 
     * Used to set up the \c GRXFSIZ register when the hardware is configured.
     * 
     * \see #setRxFifoSz
     */
    const uint32_t                          m_rxFifoSzInWords;

    /** @brief Reference to the USB Device Mode Callback Object. */
    const UsbDeviceViaSTM32F4 *             m_usbDevice;

    /** @brief USB Mode (Host vs. Device Mode). */
    DeviceMode_e                            m_mode;

    static const irq_handler_t m_irq_handler[];   
    static const irq_handler_t m_otgirq_handler[];

/*******************************************************************************
 * Private Methods
 ******************************************************************************/
private:
    /**
     * @brief Disable the USB Core Interrupts.
     * 
     * Masks all USB Core IRQs by setting the \c GINT Bit in the \c GAHBCFG register.
     */
    void disableInterrupt(void) const {
        this->m_usbCore->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;
    }

    /**
     * @brief Enables the USB Core Interrupts.
     * 
     * Masks all USB Core IRQs by setting the \c GINT Bit in the \c GAHBCFG register.
     * 
     * \attention Before unmasking the Interrupts, all pending IRQs are cleared.
     */
    void enableInterrupt(void) const {
        this->m_usbCore->GINTSTS = 0xffffffff;
        this->m_usbCore->GAHBCFG |= USB_OTG_GAHBCFG_GINT;
    }

    void    stopPhy(void) const;
    void    startPhy(void) const;

    /**
     * @brief Starts the USB Transceiver.
     * 
     * Starts the USB Transceiver by setting the \c PWRDWN bit in the \c GCCFG register.
     */
    void startTransceiver(void) const {
        this->m_usbCore->GCCFG |= USB_OTG_GCCFG_PWRDWN;
    }

    /**
     * @brief Stops the USB Transceiver.
     * 
     * Stops the USB Transceiver by clearing the \c PWRDWN bit in the \c GCCFG register.
     */
    void stopTransceiver(void) const {
        this->m_usbCore->GCCFG &= ~USB_OTG_GCCFG_PWRDWN;
    }

    /**
     * @brief Set-up the Rx FIFO Size.
     * 
     * Sets up the Rx FIFO size in the \c GRXFSIZ register.
     * 
     * @param p_rxFifoSzInWords Rx FIFO Size in Words.
     *  \pre The \p p_rxFifoSzInWords must be between 16 and 256 words.
     */
    constexpr void setRxFifoSz(const uint16_t p_rxFifoSzInWords) const {
        this->m_usbCore->GRXFSIZ = p_rxFifoSzInWords;
    }

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
 * Forward declare and typedef the instance specific types for the USB cores
 ******************************************************************************/
template<
    intptr_t UsbT,
    typename GpioPinT,
    typename RccT,
    typename NvicT
>
class UsbCoreViaSTM32F4FromAddressPointerT;

/***************************************************************************//**
 * \brief Utility class to create a UsbCoreViaSTM32F4 object from an address.
 * 
 * This class is used to create a ::usb::stm32f4::UsbCoreViaSTM32F4 object from
 * the adress to the USB Hardware's Registers.
 * 
 ******************************************************************************/
template<
    intptr_t Address,
    typename GpioPinT /* = gpio::GpioPin */,
    typename RccT /* = devices::RccViaSTM32F4 */,
    typename NvicT /* = devices::NvicViaSTM32F4*/
>
class UsbCoreViaSTM32F4FromAddressPointerT : public EngineT<Address>, public UsbCoreViaSTM32F4 {
private:
    /** @brief Reference to the NVIC.  */
    NvicT &     m_nvic;
    /** @brief Reference to the RCC. */
    RccT &      m_rcc;
    /** @brief Reference to the D- Pin. */
    GpioPinT &  m_pinDm;
    /** @brief Reference to the D+ Pin. */
    GpioPinT &  m_pinDp;
    /** @brief Reference to the Vbus Pin. */
    GpioPinT &  m_pinVbus;
    /** @brief Reference to the GND / ID Sense Pin. */
    GpioPinT &  m_pinId;

public:
    /**
     * @brief Construct a new  UsbCoreViaSTM32F4 object from an address.
     * 
     * This constructor creates a new UsbCoreViaSTM32F4 from the address to the USB Hardware,
     * i.e. the Full Speed or High Speed core.
     * 
     * It takes care of:
     * - Setting up the Clocks with the RCC.
     * - Setting up the I/O Pins.
     * - Inititizing the USB Hardware via UsbCoreViaSTM32F4::initialize.
     * - Setting up the Interrupts with the NVIC.
     * 
     * \see UsbCoreViaSTM32F4GpioHelperT::initalize
     * \see UsbCoreViaSTM32F4::initialize
     * 
     * @param p_nvic Reference to the NVIC object.
     * @param p_rcc Reference to the RCC object.
     * @param p_pinDm GPIO Pin used for the USB D+ Line.
     * @param p_pinDp GPIO Pin used for the USB D- Line.
     * @param p_pinVbus GPIO Pin used for the Vbus Line.
     * @param p_pinId GPIO Pin used for the GND / ID Sense Line.
     * @param p_rxFifoSzInWords Size of the Rx FIFO in Words (4 Bytes).
     */
    UsbCoreViaSTM32F4FromAddressPointerT(NvicT &p_nvic, RccT &p_rcc,
      GpioPinT &p_pinDm, GpioPinT &p_pinDp, GpioPinT &p_pinVbus, GpioPinT &p_pinId, uint32_t p_rxFifoSzInWords = 128)
        : UsbCoreViaSTM32F4(reinterpret_cast<USB_OTG_GlobalTypeDef *>(this->m_engineType), this->m_engineType + USB_OTG_PCGCCTL_BASE, p_rxFifoSzInWords), m_nvic(p_nvic),
            m_rcc(p_rcc), m_pinDm(p_pinDm), m_pinDp(p_pinDp), m_pinVbus(p_pinVbus), m_pinId(p_pinId)
    {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

        p_pinDm.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_pinDp.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_pinVbus.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_pinId.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));

        this->initialize();
        m_nvic.enableIrq(* static_cast<EngineT<Address> *>(this));
    }

    /**
     * @brief Destructor.
     * 
     * The destructor will reverse all operations performed by the constructure, specifially:
     * 
     * - It disables the USB IRQs with the NVIC.
     * - It resets the USB Hardware via UsbCoreViaSTM32F4::terminate.
     * - It unconfigures the I/O Pins.
     * - It disables the USB Hardware in the RCC.
     * 
     * \see UsbCoreViaSTM32F4GpioHelperT::terminate
     * \see UsbCoreViaSTM32F4::terminate
     */
    virtual ~UsbCoreViaSTM32F4FromAddressPointerT() {
        m_nvic.disableIrq(*this);
        this->terminate();

        /* TODO Disable GPIO Pins? */

        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    }
};

#if defined(USB_OTG_FS_PERIPH_BASE)
template<
    typename NvicT,
    typename RccT,
    typename GpioPinT
>
using UsbFullSpeedCoreT = UsbCoreViaSTM32F4FromAddressPointerT<USB_OTG_FS_PERIPH_BASE, GpioPinT, RccT, NvicT>;
#endif /* defined(USB_OTG_FS_PERIPH_BASE) */

#if defined(USB_OTG_HS_PERIPH_BASE)
template<
    typename NvicT,
    typename RccT,
    typename GpioPinT
>
using UsbHighSpeedCoreT = UsbCoreViaSTM32F4FromAddressPointerT<USB_OTG_HS_PERIPH_BASE, GpioPinT, RccT, NvicT>;
#endif /* defined(USB_OTG_HS_PERIPH_BASE) */

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif	/* _USBCORE_HPP_bd4e7744_c603_4e62_9377_165a9988ac70 */
