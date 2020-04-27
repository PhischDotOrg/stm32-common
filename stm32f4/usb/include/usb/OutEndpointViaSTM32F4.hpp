/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9
#define _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9

#include <stm32f4xx.h>

#include <usb/UsbHwOutEndpoint.hpp>
#include <usb/UsbOutEndpoint.hpp>
#include <usb/UsbCtrlOutEndpoint.hpp>
#include <usb/UsbBulkOutEndpoint.hpp>
#include <usb/UsbDeviceViaSTM32F4.hpp>

namespace usb {

    namespace stm32f4 {

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
     * \see ::usb::stm32f4::OutEndpointViaSTM32F4::m_irq_handler
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
     * \see ::usb::stm32f4::OutEndpointViaSTM32F4::m_irq_handler
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
    UsbOutEndpoint *                    m_endpointCallback;

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
    constexpr void registerEndpointCallback(UsbOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);
        this->m_endpointCallback = &p_endpointCallback;
    }

    constexpr void unregisterEndpointCallback(UsbOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == &p_endpointCallback);
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

    void            enableSetupPackets(const unsigned p_numPackets) const;
    void            setPacketSize(const unsigned p_packetSize) const;

    void            setup(const OutEndpointViaSTM32F4 &) const;

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4::handleRxFIFO
     ******************************************************************************/
    void            dataReceivedDeviceCallback(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e &p_dataPID) const;
    void            transferCompleteDeviceCallback(void) const;
};

template<class OutEndpointT>
class OutEndpointAdapterT : public UsbOutEndpoint {
private:
    OutEndpointT &                          m_outEndpoint;
    ::usb::UsbOutEndpoint::DataBuffer_t     m_dataBuffer;

public:
    constexpr OutEndpointAdapterT(OutEndpointT &p_outEndpoint)
        : m_outEndpoint(p_outEndpoint), m_dataBuffer { nullptr, 0 } {
    }

    virtual ~OutEndpointAdapterT() {
    }

    void transferComplete(const size_t p_numBytes) const override {
        assert(m_outEndpoint.m_endpointCallback != nullptr);
        m_outEndpoint.m_endpointCallback->transferComplete(p_numBytes);
    }

    const DataBuffer_t &getDataBuffer(void) const override {
        USB_PRINTF("OutEndpointAdapterT::%s(p_buffer=%p, p_length=%d)\r\n", __func__, this->m_dataBuffer.m_buffer, this->m_dataBuffer.m_numWords * 4);

        return this->m_dataBuffer;
    };

    void setDataBuffer(uint32_t * const p_buffer, size_t p_length) {
        this->m_dataBuffer.m_buffer = p_buffer;
        this->m_dataBuffer.m_numWords = p_length / sizeof(uint32_t);
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
class CtrlOutEndpointViaSTM32F4 {
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
    OutEndpointViaSTM32F4 &     m_outEndpoint;
    UsbSetupPacket_t            m_setupPacketBuffer;

    OutEndpointAdapterT<CtrlOutEndpointViaSTM32F4>      m_usbOutEndpAdapter;
    friend class OutEndpointAdapterT<CtrlOutEndpointViaSTM32F4>;

    /***************************************************************************//**
     * @brief Callback to USB-generic Control OUT Endpoint implementation.
     ******************************************************************************/
    UsbCtrlOutEndpointT<CtrlOutEndpointViaSTM32F4> *    m_endpointCallback;

    /*******************************************************************************
     * Private Functions (IRQ Handlers)
     ******************************************************************************/
    void            handleSetupDoneIrq(void) const;
    void            handleStatusPhaseReceivedIrq(void) const;

public:
    /*******************************************************************************
     * UsbHwCtrlOutEndpoint Interface
     ******************************************************************************/
    void registerEndpointCallback(UsbCtrlOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);

        this->m_endpointCallback = &p_endpointCallback;
        this->m_outEndpoint.registerEndpointCallback(m_usbOutEndpAdapter);
    }

    void unregisterEndpointCallback(UsbCtrlOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == &p_endpointCallback);

        this->m_outEndpoint.unregisterEndpointCallback(m_usbOutEndpAdapter);
        this->m_endpointCallback = nullptr;
    }

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4
     ******************************************************************************/
    void    handleIrq(void) const;

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4::handleRxFIFO
     ******************************************************************************/
    void    setupDataReceivedDeviceCallback(const size_t p_numBytes);
    void    setupCompleteDeviceCallback(void) const;

    constexpr CtrlOutEndpointViaSTM32F4(OutEndpointViaSTM32F4 &p_outEndpoint)
      : m_outEndpoint(p_outEndpoint), m_setupPacketBuffer {}, m_usbOutEndpAdapter(*this), m_endpointCallback(nullptr) {
          this->m_outEndpoint.m_usbDevice.registerEndpoint(*this);
    };

    ~CtrlOutEndpointViaSTM32F4() {
        this->m_outEndpoint.m_usbDevice.unregisterEndpoint(*this);
    }

    void setDataStageBuffer(uint32_t * const p_buffer, const size_t p_length) {
        USB_PRINTF("CtrlOutEndpointViaSTM32F4::%s(p_buffer=%p, p_length=%d)\r\n", __func__, p_buffer, p_length);

        m_usbOutEndpAdapter.setDataBuffer(p_buffer, p_length);
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
class BulkOutEndpointViaSTM32F4 {
private:
    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/
    OutEndpointViaSTM32F4 &                         m_outEndpoint;
    OutEndpointAdapterT<BulkOutEndpointViaSTM32F4>  m_usbOutEndpAdapter;
    friend class OutEndpointAdapterT<BulkOutEndpointViaSTM32F4>;

    /***************************************************************************//**
     * @brief Callback to USB-generic Bulk OUT Endpoint implementation.
     ******************************************************************************/
    UsbBulkOutEndpointT<BulkOutEndpointViaSTM32F4> * m_endpointCallback;
    
public:
    constexpr BulkOutEndpointViaSTM32F4(OutEndpointViaSTM32F4 &p_outEndpoint)
      : m_outEndpoint(p_outEndpoint), m_usbOutEndpAdapter(*this), m_endpointCallback(nullptr) {

    };

    ~BulkOutEndpointViaSTM32F4() {

    }

    /*******************************************************************************
     * UsbHwOutEndpoint Interface
     ******************************************************************************/
    constexpr void registerEndpointCallback(UsbBulkOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == nullptr);

        this->m_endpointCallback = &p_endpointCallback;
        this->m_outEndpoint.registerEndpointCallback(m_usbOutEndpAdapter);
    }

    constexpr void unregisterEndpointCallback(UsbBulkOutEndpoint &p_endpointCallback) {
        assert(this->m_endpointCallback == &p_endpointCallback);

        this->m_outEndpoint.unregisterEndpointCallback(m_usbOutEndpAdapter);
        this->m_endpointCallback = nullptr;
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

    void setDataBuffer(uint32_t * const p_buffer, size_t p_length) {
        m_usbOutEndpAdapter.setDataBuffer(p_buffer, p_length);
    };
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
