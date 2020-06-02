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

/*******************************************************************************
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

    typedef enum Interrupt_e : uint32_t {
        e_None = 0,
        e_TransferCompleted         = USB_OTG_DOEPINT_XFRC,
        e_EndpointDisabled          = USB_OTG_DOEPINT_EPDISD,
        e_SetupPhaseDone            = USB_OTG_DOEPINT_STUP,
        e_OutTokenWhileDisabled     = USB_OTG_DOEPINT_OTEPDIS,
        /* These are not in the Header files but in the Data Sheed */
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
        Interrupt_t     m_irq;
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
    OutEndpointViaSTM34F4Callback *                    m_endpointCallback;

    /*******************************************************************************
     * Private Functions (IRQ Handlers)
     ******************************************************************************/
    void                handleTransferCompleteIrq(void) const;

public:
    OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber);
    ~OutEndpointViaSTM32F4();

    /*******************************************************************************
     * UsbHwOutEndpoint Interface
     ******************************************************************************/
    void registerEndpointCallback(OutEndpointViaSTM34F4Callback &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);
        this->m_endpointCallback = &p_endpointCallback;
    }

    void unregisterEndpointCallback(void) {
        assert(this->m_endpointCallback != nullptr);
        this->m_endpointCallback = nullptr;
    }

    void setup(const UsbDeviceViaSTM32F4::EndpointType_e p_endpointType) const;

    /**
     * @brief Get the Endpoint Number.
     * 
     * Returns ::m_endpointNumber.
     * 
     * @return constexpr unsigned Endpoint Number without USB Direction encoding.
     */
    constexpr unsigned getEndpointNumber(void) const {
        return this->m_endpointNumber;
    }

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4
     ******************************************************************************/
    void            handleIrq(void) const;

    void            disable(void) const;
    void            enable(void) const;

    void            setPacketSize(const unsigned p_packetSize) const;

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4::handleRxFIFO
     ******************************************************************************/
    void            dataReceivedDeviceCallback(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e &p_dataPID) const;
    void            transferCompleteDeviceCallback(void) const;
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

/*******************************************************************************
 *
 ******************************************************************************/
class OutEndpointViaSTM34F4Callback {
public:
    typedef struct DataBuffer_s {
        uint32_t *  m_buffer;
        size_t      m_numWords;
    } DataBuffer_t;

protected:
    OutEndpointViaSTM32F4   m_outEndpoint;
    size_t                  m_transmitLength;
    DataBuffer_t            m_dataBuffer;

public:
    OutEndpointViaSTM34F4Callback(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber)
      : m_outEndpoint(p_usbDevice, p_endpointNumber), m_transmitLength(0), m_dataBuffer { nullptr, 0 } {
        /* Nothing to do. */
    }

    virtual ~OutEndpointViaSTM34F4Callback() {};

    void packetReceived(const size_t p_numBytes) {
        this->m_transmitLength += p_numBytes;
    };

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
    const DataBuffer_t &getDataBuffer(void) const {
        USB_PRINTF("OutEndpointViaSTM34F4Callback::%s(p_buffer=%p, p_length=%d)\r\n", __func__, this->m_dataBuffer.m_buffer, this->m_dataBuffer.m_numWords * 4);

        return this->m_dataBuffer;
    };

    /**
     * @brief Set the Data Buffer object
     * 
     * The device-independent layers use this method to set up the Endpoint's Data Buffer,
     * depending on the state of the system (e.g. Data OUT Stage in Control Request).
     * 
     * @param p_buffer Address of the Data Buffer.
     * @param p_length Size of the Data Buffer (in Bytes).
     */
    void setDataBuffer(uint32_t * const p_buffer, size_t p_length) {
        this->m_dataBuffer.m_buffer = p_buffer;
        this->m_dataBuffer.m_numWords = p_length / sizeof(uint32_t);
    }

    virtual void transferComplete(const size_t p_numBytes) const = 0;
};

/*******************************************************************************
 *
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
        OutEndpointViaSTM32F4::Interrupt_t          m_irq;
        CtrlOutEndpointViaSTM32F4::irq_handler_fn   m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/
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

    /***************************************************************************//**
     * @brief Callback to USB-generic Control OUT Endpoint implementation.
     ******************************************************************************/
    UsbCtrlOutEndpointT<CtrlOutEndpointViaSTM32F4> &    m_endpointCallout;

    /*******************************************************************************
     * Private Functions (IRQ Handlers)
     ******************************************************************************/
    void            handleSetupDoneIrq(void) const;
    void            handleStatusPhaseReceivedIrq(void) const;

public:
    CtrlOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, UsbCtrlOutEndpointT<CtrlOutEndpointViaSTM32F4> &p_endpointCallout)
      : OutEndpointViaSTM34F4Callback(p_usbDevice, 0), m_setupPacketBuffer {}, m_endpointCallout(p_endpointCallout) {
          p_endpointCallout.registerHwEndpoint(*this);
          this->m_outEndpoint.registerEndpointCallback(*this);
          this->m_outEndpoint.m_usbDevice.registerCtrlEndpoint(*this);
    };

    ~CtrlOutEndpointViaSTM32F4() {
        this->m_outEndpoint.m_usbDevice.unregisterCtrlEndpoint();
        this->m_outEndpoint.unregisterEndpointCallback();
        this->m_endpointCallout.unregisterHwEndpoint();
    }

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4
     ******************************************************************************/
    void    handleIrq(void) const;

    void    enableSetupPackets(const unsigned p_numPackets) const;

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4::handleRxFIFO
     ******************************************************************************/
    void    setupDataReceivedDeviceCallback(const size_t p_numBytes);
    void    setupCompleteDeviceCallback(void) const;

    void setDataStageBuffer(uint32_t * const p_buffer, const size_t p_length) {
        USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s(p_buffer=%p, p_length=%d)\r\n", __func__, p_buffer, p_length);

        this->setDataBuffer(p_buffer, p_length);
    }

    /*******************************************************************************
     * OutEndpointViaSTM34F4Callback Interface
     ******************************************************************************/
    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
class BulkOutEndpointViaSTM32F4 : public OutEndpointViaSTM34F4Callback {
private:
    /***************************************************************************//**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     ******************************************************************************/
    UsbBulkOutEndpointT<BulkOutEndpointViaSTM32F4> & m_endpointCallout;
    
public:
    BulkOutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, UsbBulkOutEndpointT<BulkOutEndpointViaSTM32F4> &p_endpointCallout, const unsigned p_endpointNumber)
      : OutEndpointViaSTM34F4Callback(p_usbDevice, p_endpointNumber), m_endpointCallout(p_endpointCallout) {
        m_endpointCallout.registerHwEndpoint(*this);
        m_outEndpoint.registerEndpointCallback(*this);
    };

    ~BulkOutEndpointViaSTM32F4() {
        m_outEndpoint.unregisterEndpointCallback();
        m_endpointCallout.unregisterHwEndpoint();
    }

    /*******************************************************************************
     * UsbHwBulkOutEndpoint Interface
     ******************************************************************************/
    void disable(void) const {
        this->m_outEndpoint.disable();
    }

    void enable(void) const {
        this->m_outEndpoint.enable();
    }

    void setup(void) const {
        this->m_outEndpoint.setup(UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk);
    }

    void transferComplete(const size_t p_numBytes) const override {
        m_endpointCallout.transferComplete(p_numBytes);
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
