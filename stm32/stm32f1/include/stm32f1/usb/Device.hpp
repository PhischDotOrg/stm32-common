/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291
#define __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291

#include <stm32/Engine.hpp>

#include <stm32f4xx.h>

#include <usb/UsbDevice.hpp>

#include <array>
#include <cassert>

extern "C" {
    extern char UsbBufBegin;
};

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

class InEndpoint;
class OutEndpoint;
class CtrlOutEndpoint;

struct USBEndpt {
    USBEndpt(USBEndpt &) = delete;
    USBEndpt() = default;

    volatile uint16_t   EPxR;
protected:
    uint16_t            reserved {};
};
static_assert(sizeof(struct USBEndpt) == 4);
using USBEndpt_TypeDef = struct USBEndpt;

/*****************************************************************************/

struct EndpointBufferDescriptor_s {
    uint16_t    m_txAddr;
    uint16_t    m_padding_0;
    uint16_t    m_txCount;
    uint16_t    m_padding_1;
    uint16_t    m_rxAddr;
    uint16_t    m_padding_2;
    uint16_t    m_rxCount;
    uint16_t    m_padding_3;
};
// static_assert(sizeof(UsbMem) == 4);
static_assert(sizeof(EndpointBufferDescriptor_s) == 16);

/*****************************************************************************/
class Device : public ::usb::UsbHwDevice {
    static constexpr unsigned   m_maxEndpoints = 8;
    static constexpr unsigned   m_maxOutEndpoints = 4;
    static constexpr unsigned   m_maxInEndpoints = 4;
    static_assert((m_maxInEndpoints + m_maxOutEndpoints) <= m_maxEndpoints);

    USB_TypeDef &      m_usbCore;

    InEndpoint *        m_inEndpoints[m_maxInEndpoints] {};
    OutEndpoint *       m_outEndpoints[m_maxOutEndpoints] {};
    CtrlOutEndpoint *   m_ctrlOutEndpoint {};
    alignas(8) static struct EndpointBufferDescriptor_s m_bufferDescriptorTable[m_maxEndpoints] __attribute__((section(".usbbuf")));

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

    void handleResetIrq(const uint16_t p_istr) const;
    void handleCorrectTransferIrq(const uint16_t p_istr) const;
    void handleErrorIrq(const uint16_t p_istr) const;
    void handlePacketMemOverrunIrq(const uint16_t p_istr) const;
    void handleEndpointIrq(unsigned p_endpointNo, unsigned p_direction) const;

    void enableFunction(void) const;
    void disableFunction(void) const;

    typedef void (stm32::f1::usb::Device::*irq_handler_fn)(const uint16_t p_istr) const;

    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    using irq_handler_table_t = std::array<const irq_handler_t, 4>;

    static constexpr 
    irq_handler_table_t m_irq_handler { {
        { .m_irq = Interrupt_e::e_CorrectTransfer,  .m_fn = &Device::handleCorrectTransferIrq },
        { .m_irq = Interrupt_e::e_Reset,            .m_fn = &Device::handleResetIrq },
        { .m_irq = Interrupt_e::e_Error,            .m_fn = &Device::handleErrorIrq },
        { .m_irq = Interrupt_e::e_PacketMemOverrun, .m_fn = &Device::handlePacketMemOverrunIrq }
    } };

    static constexpr uint16_t getIrqMask(uint16_t p_mask, irq_handler_table_t::const_iterator p_cur, irq_handler_table_t::const_iterator p_end);

    void setBufferTable(const void * const p_bufferTable) const;

public:
    Device(Device &) = delete;

    constexpr Device(USB_TypeDef * const p_usbCore)
      : ::usb::UsbHwDevice(DeviceSpeed_e::e_UsbFullSpeed), m_usbCore(*p_usbCore) {
    }

    void handleIrq(void) const;

    void start(void) const override;
    void stop(void) const override;

    void setAddress(uint8_t p_address) const override;

    void reset(void) const;

    void
    registerCtrlEndpoint(CtrlOutEndpoint &p_endpoint) {
        assert(this->m_ctrlOutEndpoint == nullptr);
        this->m_ctrlOutEndpoint = &p_endpoint;
    }

    void
    unregisterCtrlEndpoint(void) {
        this->m_ctrlOutEndpoint = nullptr;
    }

    void
    registerInEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint) {
        assert(p_endpointNumber < this->m_maxInEndpoints);
        assert(this->m_inEndpoints[p_endpointNumber] == nullptr);

        this->m_inEndpoints[p_endpointNumber] = &p_endpoint;
    }

    void
    unregisterInEndpoint(const unsigned p_endpointNumber) {
        assert(p_endpointNumber < this->m_maxInEndpoints);
        assert(this->m_inEndpoints[p_endpointNumber] != nullptr);

        this->m_inEndpoints[p_endpointNumber] = nullptr;
    }

    void
    registerOutEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint) {
        assert(p_endpointNumber < this->m_maxOutEndpoints);
        assert(this->m_outEndpoints[p_endpointNumber] == nullptr);

        this->m_outEndpoints[p_endpointNumber] = &p_endpoint;
    }

    void
    unregisterOutEndpoint(const unsigned p_endpointNumber) {
        assert(p_endpointNumber < this->m_maxOutEndpoints);
        assert(this->m_outEndpoints[p_endpointNumber] != nullptr);

        this->m_outEndpoints[p_endpointNumber] = nullptr;
    }

    using EndptBufDescr_t = struct EndpointBufferDescriptor_s;
    static constexpr
    EndptBufDescr_t &    
    getEndptBufferDescr(unsigned p_endptPhysIdx) {
        assert(p_endptPhysIdx < m_maxEndpoints);

        return m_bufferDescriptorTable[p_endptPhysIdx];
    }

    USBEndpt_TypeDef &
    getHwEndpt(const unsigned p_endptPhysIdx) const {
        USBEndpt_TypeDef * const hwEndpoints = reinterpret_cast<USBEndpt_TypeDef *>(&m_usbCore);

        return hwEndpoints[p_endptPhysIdx];
    }

    static constexpr uint16_t
    mapHostToPeripheral(const void * const p_hostAddr) {
        const auto offset = static_cast<const char *>(p_hostAddr) - &UsbBufBegin;
        return offset / sizeof(uint16_t);
    }
};

/*****************************************************************************/

template<
    intptr_t Address,
    typename GpioPinT /* = gpio::GpioPin */,
    typename RccT /* = devices::RccViaSTM32F4 */,
    typename NvicT /* = devices::NvicViaSTM32F4*/
>
class DeviceT : public EngineT<Address>, public Device {
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
    DeviceT(DeviceT &) = delete;

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
     * \see DeviceT::initalize
     *
     * @param p_nvic Reference to the NVIC object.
     * @param p_rcc Reference to the RCC object.
     * @param p_pinDm GPIO Pin used for the USB D+ Line.
     * @param p_pinDp GPIO Pin used for the USB D- Line.
     * @param p_rxFifoSzInWords Size of the Rx FIFO in Words (4 Bytes).
     */
    DeviceT(NvicT &p_nvic, RccT &p_rcc,
      GpioPinT &p_pinDm, GpioPinT &p_pinDp, uint32_t /* p_rxFifoSzInWords */ = 128)
        : Device(reinterpret_cast<USB_TypeDef *>(this->m_engineType)), m_nvic(p_nvic),
            m_rcc(p_rcc), m_pinDm(p_pinDm), m_pinDp(p_pinDp)
    {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

        p_pinDm.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
        p_pinDp.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));

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
    virtual ~DeviceT() {
        m_nvic.disableIrq(*this);

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
using UsbFullSpeedCoreT = DeviceT<USB_BASE, GpioPinT, RccT, NvicT>;
#endif /* defined(USB_BASE) */
/*****************************************************************************/

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_PERIPHERAL_HPP_AE22FD88_A945_4087_BA41_9201E817D291 */
