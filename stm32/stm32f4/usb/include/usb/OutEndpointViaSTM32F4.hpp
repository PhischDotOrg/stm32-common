/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9
#define _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9

#include <stm32f4xx.h>

#include <usb/UsbDeviceViaSTM32F4.hpp>

#include <usb/UsbOutEndpoint.hpp>

#include <cassert>
#include <cstddef>

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/***************************************************************************//**
 * @brief OUT Endpoint driver for STM32F4
 *
 * This class handles all OUT Endpoint specific aspects of the STM32F4 USB Core.
 * 
 * It implements the interface to the device-independent USB layer.
 * 
 * It also exposes an Interface to ::usb::stm32f4::UsbDeviceViaSTM32F4 to handle
 * callbacks from the USB Device Class.
 * 
 ******************************************************************************/
class OutEndpointViaSTM32F4 : public ::usb::UsbHwOutEndpoint {
    /**
     * \brief Private Typedef for the Endpoint IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef void (stm32::usb::OutEndpointViaSTM32F4::*irq_handler_fn)() const;

    /*******************************************************************************
     * Private Functions (IRQ Handlers)
     ******************************************************************************/
    void handleTransferCompleteIrq(void) const;

protected:
    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    /**
     * @brief Private enum / typedef for OUT Endpoint IRQs.
     * 
     * \see #m_irq_handler
     */
    typedef enum Interrupt_e : uint32_t {
        e_None = 0,
        e_TransferCompleted         = USB_OTG_DOEPINT_XFRC,
        e_EndpointDisabled          = USB_OTG_DOEPINT_EPDISD,
        e_SetupPhaseDone            = USB_OTG_DOEPINT_STUP,
        e_OutTokenWhileDisabled     = USB_OTG_DOEPINT_OTEPDIS,
        /* These are not in the Header files but in the Data Sheet */
        e_StatusPhaseReceived       = (1 << 5),
        e_OutPacketError            = (1 << 8),
        e_BabbleError               = (1 << 12),
        e_NakInput                  = (1 << 13),
        /* Couldn't find these in the Data Sheet, but they're in the Header Files */
        e_BackToBackSetupPacket     = USB_OTG_DOEPINT_B2BSTUP,
        e_NotYet                    = USB_OTG_DOEPINT_NYET,
    } Interrupt_t;

    /**
     * @brief Typedef for IRQ Handler.
     */
    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    using irq_handler_table_t = std::array<const irq_handler_t, 1>;

    /***************************************************************************//**
     * @brief OUT Endpoint Interrupt Handlers.
     *
     * Table of interrupt handlers. Is handled in order from first to last, i.e.
     * functions listed earlier are handled before the functions listed later.
     *
     * \see ::usb::stm32f4::OutEndpointViaSTM32F4::handleIrq.
     ******************************************************************************/
    static constexpr
    irq_handler_table_t m_irq_handler { {
        { .m_irq = Interrupt_e::e_TransferCompleted,    .m_fn = &OutEndpointViaSTM32F4::handleTransferCompleteIrq }
    } };

    /**
     * @brief Reference to the STM32F4 USB Device Driver object.
     * 
     */
    UsbDeviceViaSTM32F4 & m_usbDevice;

    /**
     * @brief Callback to USB-generic Control OUT Endpoint implementation.
     * 
     * This object handles the _Setup Complete_ as well as _Transfer Complete_
     * events.
     * 
     * \see #transferComplete
     * \see #handleSetupDoneIrq
     */
    ::usb::UsbOutEndpoint & m_endpointCallout;

    /**
     * @brief OUT Endpoint Number
     * 
     * This is the Endpoint's Number relative to the USB Hardware, i.e. without
     * USB direction encoding.
     */
    const unsigned m_endpointNumber;

    /**
     * @brief STM32F4 HAL Structure Pointer for Endpoint Hardware Access.
     *
     * The Class Code uses this pointer to access the Hardware Registers used
     * to control the endpoint.
     */
    USB_OTG_OUTEndpointTypeDef * const  m_endpoint;

    /**
     * @brief Access to the Front of the Endpoint's Rx FIFO.
     * 
     * The Class Code uses this address to read / pop data elements from the
     * USB Hardware's Rx FIFO front.
     */
    volatile uint32_t * const           m_fifoAddr;

public:
    /**
     * @name Constructor/Destructor.
     * 
     */
///@{ 
    /***************************************************************************//**
    * \brief Constructor.
    * 
    * Constructs a new ::usb::stm32f4::OutEndpointViaSTM32F4 with the given endpoint
    * number and registers it as a callback receiver with the provided
    * ::usb::stm32f4::UsbDeviceViaSTM32F4 object.
    * 
    * \param p_usbDevice Object that represents the STM32F4 USB Device Hardware Driver.
    * \param p_endpointNumber Endpoint Number without USB direction encoding.
    *
    * \see UsbDeviceViaSTM32F4::registerOutEndpoint
    ******************************************************************************/
    OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, ::usb::UsbOutEndpoint &p_endpointCallout, const unsigned p_endpointNumber)
      : m_usbDevice(p_usbDevice),
        m_endpointCallout(p_endpointCallout),
        m_endpointNumber(p_endpointNumber),
        m_endpoint(reinterpret_cast<USB_OTG_OUTEndpointTypeDef *>(p_usbDevice.getBaseAddr() + USB_OTG_OUT_ENDPOINT_BASE + (p_endpointNumber * USB_OTG_EP_REG_SIZE))),
        m_fifoAddr(reinterpret_cast<uint32_t *>(p_usbDevice.getBaseAddr() + USB_OTG_FIFO_BASE + (p_endpointNumber * USB_OTG_FIFO_SIZE)))
    {
        m_usbDevice.registerOutEndpoint(this->getEndpointNumber(), *this);
        m_endpointCallout.registerHwOutEndpoint(*this);
    }

    /***************************************************************************//**
     * @brief Destructor.
     * 
     * Destructs the object and unregisters it at the STM32F4 USB Device Hardware
     * driver referred to by #m_usbDevice.
     * 
     * \see UsbDeviceViaSTM32F4::unregisterEndpoint
     ******************************************************************************/
    virtual ~OutEndpointViaSTM32F4() {
        m_endpointCallout.unregisterHwOutEndpoint();
        this->m_usbDevice.unregisterOutEndpoint(this->getEndpointNumber());
    }
///@}

    /**
     * @name Interface to the Endpoint-type specific Classes.
     * 
     * Interface to the CtrlOutEndpointViaSTM32F4 and BulkOutEndpointViaSTM32F4 concrete
     * Endpoint classes.
     * 
     * These methods are used by the Endpoint-type specific sub-classes, e.g. to register
     * themselves as a call-out handler or to setup/enable/disable the actual hardware.
     */
///@{ 
    void setup(const UsbDeviceViaSTM32F4::EndpointType_e p_endpointType) const;
    void disable(void) const;
    void enable(void) const;
///@}

    /**
     * @name Interface to the USB Device Driver.
     * 
     * These methods provide an interface to the UsbDeviceViaSTM32F4 Device Driver class.
     */
///@{ 
    /**
     * @brief Get the Endpoint Number.
     * 
     * Returns #m_endpointNumber.
     * 
     * @return constexpr unsigned Endpoint Number without USB Direction encoding.
     */
    constexpr unsigned getEndpointNumber(void) const {
        return this->m_endpointNumber;
    }

    void handleIrq(void) const;
    void setPacketSize(const unsigned p_packetSize) const;
    void dataReceivedDeviceCallback(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e &p_dataPID) const;
    void transferCompleteDeviceCallback(void) const;
///@}

    void setData(unsigned /* p_dtog */) const override { /* FIXME Implement this */ };
    bool getData(void) const override { /* FIXME Implement this */ return false; };
    void nack(void) const override;
    void stall(void) const override;
    void ack(void) const override;
};

/***************************************************************************//**
 * @brief Control OUT Endpoint Driver for STM32F4.
 * 
 * This class handles the Control OUT Endpoint specific aspects of the STM32F4
 * USB Device Hardware.
 ******************************************************************************/
class CtrlOutEndpointViaSTM32F4 : public OutEndpointViaSTM32F4 {
private:
    /**
     * \brief Private Typedef for the Endpoint IRQ Handlers.
     * 
     * \see ::usb::stm32f4::CtrlOutEndpointViaSTM32F4::m_irq_handler
     */
    typedef void (stm32::usb::CtrlOutEndpointViaSTM32F4::*irq_handler_fn)() const;

    /**
     * @brief Private Data Type to construct the Table of IRQ Handlers.
     * 
     * \see ::usb::stm32f4::OutEndpointViaSTM32F4::m_irq_handler
     */
    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        OutEndpointViaSTM32F4::Interrupt_t          m_irq;
        /** @brief Pointer to IRQ Handler Member function. */
        CtrlOutEndpointViaSTM32F4::irq_handler_fn   m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /***************************************************************************//**
     * @name Interrupt Handlers.
     * 
     * These methods implement interrupt handlers for the Control OUT Endpoint.
     * 
     * \see #handleIrq
     * \see #m_irq_handler
     ******************************************************************************/
///@{
    void            handleSetupDoneIrq(void) const;
    void            handleStatusPhaseReceivedIrq(void) const;
///@}

public:
    /***************************************************************************//**
     * @name Constructor/Destructor.
     *
     ******************************************************************************/
///@{
    /**
     * @brief Constructor.
     * 
     * This constructor registers the object as a callback with:
     * 
     * - UsbCtrlOutEndpointT::registerHwEndpoint (referred to by #m_endpointCallout).
     * - OutEndpointViaSTM32F4::registerEndpointCallback (referred to by #m_outEndpoint).
     * - UsbDeviceViaSTM32F4::registerCtrlEndpoint.
     * 
     * @param p_usbDevice Reference to UsbDeviceViaSTM32F4 object that handles the USB Device
     *  Mode specific hardware access.
     * @param p_endpointCallout Reference to a UsbCtrlOutEndpointT object that handles the
     *   hardware-independent request.
     * 
     * @see m_endpointCallout
     */
    CtrlOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, ::usb::UsbCtrlOutEndpoint &p_endpointCallout)
      : OutEndpointViaSTM32F4(p_usbDevice, p_endpointCallout, 0) {
          this->m_usbDevice.registerCtrlEndpoint(*this);
    };

    /**
     * @brief Destructor.
     * 
     * Unregisters the object from all callbacks.
     * 
     * \see #CtrlOutEndpointViaSTM32F4
     */
    ~CtrlOutEndpointViaSTM32F4() {
        this->m_usbDevice.unregisterCtrlEndpoint();
        this->m_endpointCallout.unregisterHwOutEndpoint();
    }
///@}

    /***************************************************************************//**
     * @name Interface to UsbDeviceViaSTM32F4.
     * 
     * These methods implement an interface to UsbDeviceViaSTM32F4.
     * 
     * \see UsbDeviceViaSTM32F4::m_ctrlOutEndpoint
     ******************************************************************************/
///@{
    void    handleIrq(void) const;

    void    enableSetupPackets(const unsigned p_numPackets) const;
    void    setupDataReceivedDeviceCallback(const size_t p_numBytes);
    void    setupCompleteDeviceCallback(void) const;
///@}
};

/***************************************************************************//**
 * @brief Bulk OUT Endpoint Driver for STM32F4.
 * 
 * This class handles the Bulk OUT Endpoint specific aspects of the STM32F4
 * USB Device Hardware.
 ******************************************************************************/
class BulkOutEndpointViaSTM32F4 : public OutEndpointViaSTM32F4 {
public:
    /***************************************************************************//**
     * @name Constructor/Destructor.
     *
     ******************************************************************************/
///@{

    /**
     * @brief Constructor.
     * 
     * This constructor registers the object as a callback with:
     * 
     * - UsbBulkOutEndpointT::registerHwEndpoint (referred to by #m_endpointCallout).
     * - OutEndpointViaSTM32F4::registerEndpointCallback (referred to by #m_outEndpoint).
     * 
     * @param p_usbDevice Reference to UsbDeviceViaSTM32F4 object that handles the USB Device
     *  Mode specific hardware access.
     * @param p_endpointCallout Reference to a UsbBulkOutEndpointT object that handles the
     *   hardware-independent request.
     *
     * @param p_endpointNumber Number of OUT Endpoint.
     * 
     * @see m_endpointCallout
     */
    BulkOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, ::usb::UsbBulkOutEndpoint &p_endpointCallout, const unsigned p_endpointNumber)
      : OutEndpointViaSTM32F4(p_usbDevice, p_endpointCallout, p_endpointNumber) {
    };
///@}

    /***************************************************************************//**
     * @name Interface to UsbBulkOutEndpointT
     *
     * These methods are an interface to the hardware-independent USB layer for
     * Bulk OUT endpoints implemented in UsbBulkOutEndpointT.
     ******************************************************************************/
///@{
    /**
     * @brief Set up the Endpoint in hardware.
     * 
     * Sets up the OUT Endpoint as a Bulk OUT Endpoint via the #m_outEndpoint reference.
     * 
     * \see OutEndpointViaSTM32F4::setup
     */
    void setup(void) const {
        OutEndpointViaSTM32F4::setup(UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk);
    }
///@}
};

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
