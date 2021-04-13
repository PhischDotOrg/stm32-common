/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291
#define __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291

#include <stm32/Engine.hpp>

#include <stm32f4xx.h>

#include <cassert>

extern "C" {
    extern char UsbBufBegin;
};

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

class Device;

/*****************************************************************************/
class Peripheral {
    USB_TypeDef &m_usbCore;

    typedef enum class Interrupt_e : uint16_t {
        e_None              = 0,
        e_CorrectTransfer   = USB_CNTR_CTRM,
        e_PacketMemOverrun  = USB_CNTR_PMAOVRM,
        e_Error             = USB_CNTR_ERRM,
        e_Wakeup            = USB_CNTR_WKUPM,
        e_Suspend           = USB_CNTR_SUSPM,
        e_Reset             = USB_CNTR_RESETM,
        e_StartOfFrame      = USB_CNTR_SOFM,
        e_ExpectedSOF       = USB_CNTR_ESOFM,
        e_All               = 0b1111111100000000
    } Interrupt_t;

    typedef void (stm32::f1::usb::Peripheral::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    const Device *  m_device;

public:
    Peripheral(USB_TypeDef * const p_usbCore)
      : m_usbCore(*p_usbCore) {
    }

    void handleIrq(void) const;

    void start(void) const;
    void stop(void) const;

    void setAddress(uint8_t p_address) const;

    void setBufferTable(const void * const p_bufferTable) const;

    void
    registerDevice(const Device &p_device) {
        assert(m_device == nullptr);

        m_device = &p_device;
    }

    void
    unregisterDevice(void) {
        assert(m_device != nullptr);

        m_device = nullptr;
    }

    uintptr_t
    getBaseAddr(void) const {
        return reinterpret_cast<uintptr_t>(&this->m_usbCore);
    }

    static /* constexpr */ uint16_t
    mapHostToPeripheral(uintptr_t p_hostAddr) {
        return static_cast<uint16_t>((p_hostAddr - reinterpret_cast<uintptr_t>(&UsbBufBegin)) / sizeof(uint16_t));
    }

protected:
    void initialize(void) const {
    }

    void terminate(void) const {
    }

private:
    static void
    delay(void) {
        for (int i = 0; i < 72; i++) {
             __NOP();
        }
    }

    void enterPwrDown(void) const;
    void exitPwrDown(void) const;

    void enterLowPowerMode(void) const;
    void exitLowPowerMode(void) const;

    void forceReset(void) const;
    void clearReset(void) const;

    void enableInterrupt(Interrupt_t p_irq) const;
    void disableInterrupt(Interrupt_t p_irq) const;
    void clearInterrupt(Interrupt_t p_irq) const;

    void enableInterrupts(void) const;

    void disableInterrupts(void) const {
        disableInterrupt(Interrupt_e::e_All);
    }

    void handleResetIrq(void) const;
    void handleCorrectTransferIrq(void) const;
    void handleErrorIrq(void) const;

    void enableFunction(void) const;
    void disableFunction(void) const;

};
/*****************************************************************************/

template<
    intptr_t Address,
    typename GpioPinT /* = gpio::GpioPin */,
    typename RccT /* = devices::RccViaSTM32F4 */,
    typename NvicT /* = devices::NvicViaSTM32F4*/
>
class PeripheralT : public EngineT<Address>, public Peripheral {
private:
    /** @brief Reference to the NVIC.  */
    NvicT &     m_nvic;
    /** @brief Reference to the RCC. */
    RccT &      m_rcc;
    /** @brief Reference to the D- Pin. */
    GpioPinT &  m_pinDm;
    /** @brief Reference to the D+ Pin. */
    GpioPinT &  m_pinDp;

public:
    /**
     * @brief Construct a new  USB Peripheral object from an address.
     *
     * This constructor creates a new Peripheral from the address to the USB Hardware.
     *
     * It takes care of:
     * - Setting up the Clocks with the RCC.
     * - Setting up the I/O Pins.
     * - Inititizing the USB Hardware via UsbCore::initialize.
     * - Setting up the Interrupts with the NVIC.
     *
     * \see PeripheralT::initalize
     *
     * @param p_nvic Reference to the NVIC object.
     * @param p_rcc Reference to the RCC object.
     * @param p_pinDm GPIO Pin used for the USB D+ Line.
     * @param p_pinDp GPIO Pin used for the USB D- Line.
     * @param p_rxFifoSzInWords Size of the Rx FIFO in Words (4 Bytes).
     */
    PeripheralT(NvicT &p_nvic, RccT &p_rcc,
      GpioPinT &p_pinDm, GpioPinT &p_pinDp, uint32_t /* p_rxFifoSzInWords */ = 128)
        : Peripheral(reinterpret_cast<USB_TypeDef *>(this->m_engineType)), m_nvic(p_nvic),
            m_rcc(p_rcc), m_pinDm(p_pinDm), m_pinDp(p_pinDp)
    {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

        p_pinDm.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_pinDp.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));

        this->initialize();
        m_nvic.enableIrq(* static_cast<EngineT<Address> *>(this));
    }

    /**
     * @brief Destructor.
     *
     * The destructor will reverse all operations performed by the constructure, specifially:
     *
     * - It disables the USB IRQs with the NVIC.
     * - It resets the USB Hardware via Peripheral::terminate.
     * - It unconfigures the I/O Pins.
     * - It disables the USB Hardware in the RCC.
     *
     * \see Peripheral::terminate
     */
    virtual ~PeripheralT() {
        m_nvic.disableIrq(*this);
        this->terminate();

        /* TODO Disable GPIO Pins? */

        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    }
};

/*****************************************************************************/
#if defined(USB_BASE)
template<
    typename NvicT,
    typename RccT,
    typename GpioPinT
>
using UsbFullSpeedCoreT = PeripheralT<USB_BASE, GpioPinT, RccT, NvicT>;
#endif /* defined(USB_BASE) */
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291 */
