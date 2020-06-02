/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9
#define _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9

#include <stm32f4xx.h>

#include <usb/UsbDeviceViaSTM32F4.hpp>

namespace usb {

    namespace stm32f4 {

        class OutEndpointViaSTM34F4Callback;

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
class OutEndpointViaSTM32F4 {
    friend class CtrlOutEndpointViaSTM32F4;

private:
    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    /**
     * \brief Private Typedef for the Endpoint IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef void (usb::stm32f4::OutEndpointViaSTM32F4::*irq_handler_fn)() const;

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
     * @brief Private Data Type to construct the Table of IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef struct irq_handler_s {
        /** @brief Interrupt to be handled. */
        Interrupt_t     m_irq;
        /** @brief Pointer to IRQ Handler Member function. */
        irq_handler_fn  m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/

    /**
     * @brief Reference to the STM32F4 USB Device Driver object.
     * 
     */
    UsbDeviceViaSTM32F4 &               m_usbDevice;

    /**
     * @brief OUT Endpoint Number
     * 
     * This is the Endpoint's Number relative to the USB Hardware, i.e. without
     * USB direction encoding.
     */
    const unsigned                      m_endpointNumber;

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

    /***************************************************************************//**
     * @brief Callback to USB-generic OUT Endpoint implementation.
     ******************************************************************************/
    OutEndpointViaSTM34F4Callback *     m_endpointCallback;

    /*******************************************************************************
     * Private Functions (IRQ Handlers)
     ******************************************************************************/
    void                handleTransferCompleteIrq(void) const;

public:
    /**
     * @name Constructor/Destructor.
     * 
     */
///@{ 
    OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber);
    ~OutEndpointViaSTM32F4();
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
    /**
     * @brief Register a Callback Object.
     * 
     * The OutEndpointViaSTM34F4Callback is used to bridge to the hardware-indepentent layer.
     * 
     * The object is notified e.g. in case of a _Transfer Complete_ Interrupt.
     * 
     * \see #handleTransferCompleteIrq
     * 
     * @param p_endpointCallback Reference to a OutEndpointViaSTM34F4Callback object.
     */
    void registerEndpointCallback(OutEndpointViaSTM34F4Callback &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);
        this->m_endpointCallback = &p_endpointCallback;
    }

    /**
     * @brief Unregisters a Callback Object.
     * 
     * Unregisters a OutEndpointViaSTM34F4Callback object by setting #m_endpointCallback to
     * \c nullptr .
     */
    void unregisterEndpointCallback(void) {
        assert(this->m_endpointCallback != nullptr);
        this->m_endpointCallback = nullptr;
    }

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
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#include <usb/UsbOutEndpoint.hpp>

/*******************************************************************************
 *
 ******************************************************************************/
namespace usb {
    namespace stm32f4 {

/***************************************************************************//**
 * @brief Interface between the OUT Endpoint driver and the type-specific classes.
 * 
 * This is an interface to bridge between the generic OutEndpointViaSTM32F4 
 * OUT endpoint driver and the Endpoint-type specific classes:
 * - CtrlOutEndpointViaSTM32F4
 * - BulkOutEndpointViaSTM32F4
 ******************************************************************************/
class OutEndpointViaSTM34F4Callback {
public:
    /**
     * @brief Structure to describe an OUT Endpoint Data Buffer.
     * 
     * This structure is used to describe the RAM Data Buffer for an OUT transfer.
     * 
     * \see getDataBuffer
     * \see setDataBuffer
     */
    typedef struct DataBuffer_s {
        /** @brief Pointer to RAM Buffer that can hold OUT Data. */
        uint32_t *  m_buffer;
        /** @brief Size of Buffer pointed to by #m_buffer in units of Words (4-Bytes). */
        size_t      m_numWords;
    } DataBuffer_t;

protected:
    /**
     * @brief Generic OUT Endpoint Driver.
     * 
     * This aggretate member is used to drive the actual OUT Endpoint hardware.
     */
    OutEndpointViaSTM32F4   m_outEndpoint;
    /**
     * @brief Counter for transmitted OUT Bytes.
     * 
     * This variable is used to count the number of bytes received in the last OUT transfer.
     * 
     * It is increased in #packetReceived and reset after #transferComplete has been called.
     */
    size_t                  m_transmitLength;

    /** @brief Data Buffer for received  OUT Data.
     * 
     * This structure is set up when #setDataBuffer is called by the hardware-independent
     * layers and is used to refer to the RAM Buffer that receives OUT Data.
     *
     */
    DataBuffer_t            m_dataBuffer;

private:
    /**
     * @name Interface to sub-classes.
     * 
     * Interface used to call into concrete sub-classes.
     * 
     */
///@{ 
    /**
     * @brief Sub-class specific Transfer Complete handler.
     * 
     * This method is implemented by concrete sub-classes handle a _Transfer Complete_
     * Interrupt.
     * 
     * \see transferComplete()
     *
     * @param p_numBytes Number of Bytes received in the last transfer.
     */
    virtual void transferComplete(const size_t p_numBytes) const = 0;
///@}

public:
    /**
     * @name Constructor/Destructor.
     * 
     */
///@{
    /**
     * @brief Constructor.
     * 
     * @param p_usbDevice Reference to UsbDeviceViaSTM32F4 object that handles the USB Device
     *  Mode specific hardware access.
     * @param p_endpointNumber Number of the endpoint.
     */
    OutEndpointViaSTM34F4Callback(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber)
      : m_outEndpoint(p_usbDevice, p_endpointNumber), m_transmitLength(0), m_dataBuffer { nullptr, 0 } {
        /* Nothing to do. */
    }

    /**
     * @brief Destructor.
     * 
     */
    virtual ~OutEndpointViaSTM34F4Callback() {};
///@}

    /**
     * @name Interface to the OUT Endpoint Handler.
     * 
     * These methods are an interface to the generic OUT Endpoint Handler class OutEndpointViaSTM32F4.
     */
///@{ 
    /**
     * @brief Handle the OUT Endpoint _Packet Received_ Event.
     * 
     * Called by OutEndpointViaSTM32F4::dataReceivedDeviceCallback when a data packet is received.
     * 
     * The main purpose is to count the number of received OUT data bytes in #m_transmitLength.
     */
    void packetReceived(const size_t p_numBytes) {
        this->m_transmitLength += p_numBytes;
    };

    /**
     * @brief Handle the OUT Endpoint _Transfer Complete_ IRQ.
     * 
     * Called by OutEndpointViaSTM32F4::handleTransferCompleteIrq when a _Transfer Comlete_ IRQ
     * occurs.
     * 
     * The call is propagated to a Endpoint-type specific handler in a concrete sub-class.
     * 
     * \see OutEndpointViaSTM34F4Callback::transferComplete(const size_t p_numBytes) const
     */
    void transferComplete(void) {
        this->transferComplete(this->m_transmitLength);

        this->m_transmitLength = 0;
    }

    /**
     * @brief Get Information on the Endpoint's Data Buffer.
     * 
     * Returns the address and length of the Endpoint's Data Buffer.
     * 
     * \see setDataBuffer
     * 
     * @return const DataBuffer_t& Structure that describes the Data Buffer's address and length.
     */
    // OutEndpointViaSTM32F4::dataReceivedDeviceCallback
    // OutEndpointViaSTM32F4::setup
    const DataBuffer_t &getDataBuffer(void) const {
        USB_PRINTF("OutEndpointViaSTM34F4Callback::%s(p_buffer=%p, p_length=%d)\r\n", __func__, this->m_dataBuffer.m_buffer, this->m_dataBuffer.m_numWords * 4);

        return this->m_dataBuffer;
    };
///@}

///@{ 
    /**
     * @brief Set the Data Buffer object
     * 
     * The device-independent layers use this method to set up the Endpoint's Data Buffer,
     * depending on the state of the system (e.g. Data OUT Stage in Control Request).
     * 
     * \see CtrlOutEndpointViaSTM32F4::setDataStageBuffer
     * \see UsbBulkOutEndpointT::registerHwEndpoint
     * 
     * @param p_buffer Address of the Data Buffer.
     * @param p_length Size of the Data Buffer (in Bytes).
     */
    void setDataBuffer(uint32_t * const p_buffer, size_t p_length) {
        this->m_dataBuffer.m_buffer = p_buffer;
        this->m_dataBuffer.m_numWords = p_length / sizeof(uint32_t);
    }
///@}
};

/***************************************************************************//**
 * @brief Control OUT Endpoint Driver for STM32F4.
 * 
 * This class handles the Control OUT Endpoint specific aspects of the STM32F4
 * USB Device Hardware.
 ******************************************************************************/
class CtrlOutEndpointViaSTM32F4 : public OutEndpointViaSTM34F4Callback {
private:
    /**
     * \brief Private Typedef for the Endpoint IRQ Handlers.
     * 
     * \see ::usb::stm32f4::CtrlOutEndpointViaSTM32F4::m_irq_handler
     */
    typedef void (usb::stm32f4::CtrlOutEndpointViaSTM32F4::*irq_handler_fn)() const;

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

    /**
     * @brief RAM Buffer for a _SETUP_ Packet.
     * 
     * RAM Buffer into which the _SETUP_ Packet is transferred by #setupDataReceivedDeviceCallback.
     * This Buffer is shared with the Hardware-independents layers, i.e. the USB Control
     * Pipe will receive a reference to this buffer to decode the packet.
     * 
     * \see handleSetupDoneIrq
     */
    UsbSetupPacket_t            m_setupPacketBuffer;

    /**
     * @brief Callback to USB-generic Control OUT Endpoint implementation.
     * 
     * This object handles the _Setup Complete_ as well as _Transfer Complete_
     * events.
     * 
     * \see #transferComplete
     * \see #handleSetupDoneIrq
     */
    UsbCtrlOutEndpointT<CtrlOutEndpointViaSTM32F4> &    m_endpointCallout;

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
    CtrlOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, UsbCtrlOutEndpointT<CtrlOutEndpointViaSTM32F4> &p_endpointCallout)
      : OutEndpointViaSTM34F4Callback(p_usbDevice, 0), m_setupPacketBuffer {}, m_endpointCallout(p_endpointCallout) {
          p_endpointCallout.registerHwEndpoint(*this);
          this->m_outEndpoint.registerEndpointCallback(*this);
          this->m_outEndpoint.m_usbDevice.registerCtrlEndpoint(*this);
    };

    /**
     * @brief Destructor.
     * 
     * Unregisters the object from all callbacks.
     * 
     * \see #CtrlOutEndpointViaSTM32F4
     */
    ~CtrlOutEndpointViaSTM32F4() {
        this->m_outEndpoint.m_usbDevice.unregisterCtrlEndpoint();
        this->m_outEndpoint.unregisterEndpointCallback();
        this->m_endpointCallout.unregisterHwEndpoint();
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

    /***************************************************************************//**
     * @name Interface to UsbCtrlOutEndpointT.
     * 
     * These methods implement an interface to UsbCtrlOutEndpointT.
     * 
     * \see UsbCtrlOutEndpointT::m_hwEndpoint
     ******************************************************************************/
///@{
    /**
     * @brief Setup the RAM Buffer for the _Data OUT_ Stage.
     * 
     * Sets up the RAM Buffer that receives the _Data OUT_ Stage of the Control Transfer
     * 
     * @param p_buffer Pointer to RAM Buffer.
     * @param p_length Length of RAM Buffer in units of Bytes.
     */
    void setDataStageBuffer(uint32_t * const p_buffer, const size_t p_length) {
        USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s(p_buffer=%p, p_length=%d)\r\n", __func__, p_buffer, p_length);

        this->setDataBuffer(p_buffer, p_length);
    }
///@}

private:
    /***************************************************************************//**
     * @name Interface to OutEndpointViaSTM34F4Callback.
     * 
     * These methods implement an interface to the OutEndpointViaSTM34F4Callback parent class.
     * 
     * \see OutEndpointViaSTM34F4Callback::transferComplete
     ******************************************************************************/
///@{
    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }
///@}
};

/***************************************************************************//**
 * @brief Bulk OUT Endpoint Driver for STM32F4.
 * 
 * This class handles the Bulk OUT Endpoint specific aspects of the STM32F4
 * USB Device Hardware.
 ******************************************************************************/
class BulkOutEndpointViaSTM32F4 : public OutEndpointViaSTM34F4Callback {
private:
    /**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     */
    UsbBulkOutEndpointT<BulkOutEndpointViaSTM32F4> & m_endpointCallout;

    /***************************************************************************//**
     * @name Interface to OutEndpointViaSTM34F4Callback.
     * 
     * These methods implement an interface to the OutEndpointViaSTM34F4Callback parent class.
     * 
     * \see OutEndpointViaSTM34F4Callback::transferComplete
     ******************************************************************************/
///@{
    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }
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
    BulkOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, UsbBulkOutEndpointT<BulkOutEndpointViaSTM32F4> &p_endpointCallout, const unsigned p_endpointNumber)
      : OutEndpointViaSTM34F4Callback(p_usbDevice, p_endpointNumber), m_endpointCallout(p_endpointCallout) {
        m_endpointCallout.registerHwEndpoint(*this);
        m_outEndpoint.registerEndpointCallback(*this);
    };

    /**
     * @brief Destructor.
     * Unregisters the object from all callbacks.
     * 
     * \see #BulkOutEndpointViaSTM32F4
     */
    ~BulkOutEndpointViaSTM32F4() {
        m_outEndpoint.unregisterEndpointCallback();
        m_endpointCallout.unregisterHwEndpoint();
    }
///@}

    /***************************************************************************//**
     * @name Interface to UsbBulkOutEndpointT
     *
     * These methods are an interface to the hardware-independent USB layer for
     * Bulk OUT endpoints implemented in UsbBulkOutEndpointT.
     ******************************************************************************/
///@{
    /**
     * @brief Disable the Endpoint.
     * 
     * Disables the OUT Endpoint via the #m_outEndpoint reference.
     * 
     * \see OutEndpointViaSTM32F4::disable
     */
    void disable(void) const {
        this->m_outEndpoint.disable();
    }

    /**
     * @brief Enables the Endpoint.
     * 
     * Enables the OUT Endpoint via the #m_outEndpoint reference.
     * 
     * \see OutEndpointViaSTM32F4::enable
     */
    void enable(void) const {
        this->m_outEndpoint.enable();
    }

    /**
     * @brief Set up the Endpoint in hardware.
     * 
     * Sets up the OUT Endpoint as a Bulk OUT Endpoint via the #m_outEndpoint reference.
     * 
     * \see OutEndpointViaSTM32F4::setup
     */
    void setup(void) const {
        this->m_outEndpoint.setup(UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk);
    }
///@}
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
