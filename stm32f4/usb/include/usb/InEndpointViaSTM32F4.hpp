/*-
 * $Copyright$
 -*/

#ifndef _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8
#define _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8

#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <stm32f4xx.h>

namespace usb {

    namespace stm32f4 {

class UsbDeviceViaSTM32F4;
class BulkInEndpointViaSTM32F4;
class CtrlInEndpointViaSTM32F4;

/***************************************************************************//**
 * \brief IN Endpoint driver for STM32F4.
 * 
 * This class handles all IN Endpoint specific aspects of the STM32F4 USB Core.
 * 
 * It implements the ::usb::UsbHwInEndpoint interface to the device-independent
 * USB  layer.
 * 
 * It also exposes an Interface to ::usb::stm32f4::UsbDeviceViaSTM32F4 to handle
 * callbacks from the USB Device Class.
 ******************************************************************************/
class InEndpointViaSTM32F4 {
private:
    /**
     * @brief Reference to the STM32F4 USB Device Driver Object.
     * 
     * Reference to an object of class ::usb::stm32f4::UsbDeviceViaSTM32F4 which
     * encapsulates all STM32F4-specific operations for USB Device mode.
     */
    UsbDeviceViaSTM32F4 &               m_usbDevice;

    /**
     * @brief IN Endpoint Number (without Direction Bit).
     * 
     * IN Endpoint Number, encoded without the Direction Bit. Can be set to a range
     * between 0 and ::usb::stm32f4::UsbDeviceViaSTM32F4::m_maxInEndpoints as it's
     * used to refer to the IN Endpoint Number within the STM32F4 USB Device Hardware.
     */
    const unsigned                      m_endpointNumber;

    /**
     *  @brief Endpoint FIFO Size in Words, i.e. units of 4 Bytes.
     */
    const size_t                        m_fifoSzInWords;
    USB_OTG_INEndpointTypeDef * const   m_endpoint;
    volatile uint32_t * const           m_fifoAddr;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    typedef void (usb::stm32f4::InEndpointViaSTM32F4::*irq_handler_fn)();

    typedef struct irq_handler_s {
        uint32_t m_irq;
        irq_handler_fn m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    typedef struct TxBuffer_s {
        bool                    m_isString;
        union TxBuffer_u {
            const uint32_t *    m_u32;
            const uint8_t *     m_u8;
            const char *        m_str;
            const char16_t *    m_str16;
            const wchar_t *     m_wstr;
        }                       m_data;
        uint8_t                 m_dataLength;
        size_t                  m_txLength;
        size_t                  m_offs;
        bool                    m_inProgress;
    } TxBuffer_t;
    TxBuffer_t  m_txBuffer;

    /*******************************************************************************
     * Private Functions
     ******************************************************************************/
    void    setNack(const bool p_nack) const;

    void    startTx(void);

    void    txData(void);
    void    txString(void);

    void    handleTxFifoEmpty(void);
    void    handleNakEffective(void);
    void    handleInTokenWhenTxFifoEmpty(void);
    void    handleTimeoutCondition(void);
    void    handleEndpointDisabled(void);
    void    handleTransferComplete(void);

    void    reset(void) const;

    void    disableIrq(void) const;
    void    enableIrq(void) const;

    void    fillTxFifo(void);
    void    markTxBufferComplete(void);
    
    void    setupEndpointType(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const;

    unsigned    getPacketSize(void) const;
    unsigned    getNumPackets(const size_t p_txLength, const size_t p_packetSz) const;

public:
    InEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber = 0);
    virtual ~InEndpointViaSTM32F4();

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4
     ******************************************************************************/
    void handleIrq(void);

    void setupTxFifoNumber(const unsigned p_fifoNumber) const {
        this->m_endpoint->DIEPCTL &= ~USB_OTG_DIEPCTL_TXFNUM_Msk;
        this->m_endpoint->DIEPCTL |= (p_fifoNumber << USB_OTG_DIEPCTL_TXFNUM_Pos) & USB_OTG_DIEPCTL_TXFNUM_Msk;
    }

    void setPacketSize(const unsigned p_packetSize) const;

    /***************************************************************************//**
     * \brief Get the IN Endpoint's Tx FIFO Size.
     * 
     * \return size_t FIFO Size in Words, i.e. units of 4 Bytes.
     ******************************************************************************/
    constexpr size_t getFifoSzInWords(void) const {
        return this->m_fifoSzInWords;
    }

    /***************************************************************************//**
     * \brief Get the IN Endpoint's Number.
     * 
     * Returns the IN Endpoint's Number. Please note that this is without direction
     * encoding, i.e. there can be both an IN and an OUT Endpoint with the same
     * number.
     * 
     * @return unsigned The IN Endpoint's Number (without Direction Bit).
     */
    constexpr unsigned getEndpointNumber(void) const {
        return this->m_endpointNumber;
    }

    /*******************************************************************************
     * Implementation of UsbHwInEndpoint Interface
     ******************************************************************************/
    void write(const uint8_t * const p_data, const size_t p_length); 
    void writeString(const ::usb::UsbStringDescriptor &p_string, const size_t p_len);

    void enable(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const;
    void disable(void) const;
};

/*******************************************************************************
 *
 ******************************************************************************/
class CtrlInEndpointViaSTM32F4 {
private:
    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/
    InEndpointViaSTM32F4 &  m_inEndpoint;

public:
    constexpr CtrlInEndpointViaSTM32F4(InEndpointViaSTM32F4 &p_inEndpoint) : m_inEndpoint(p_inEndpoint) {

    }

    ~CtrlInEndpointViaSTM32F4() {

    }

    void write(const uint8_t * const p_data, const size_t p_length) const {
        this->m_inEndpoint.write(p_data, p_length);
    };

    void writeString(const ::usb::UsbStringDescriptor &p_string, const size_t p_len) const {
        this->m_inEndpoint.writeString(p_string, p_len);
    };
};

/*******************************************************************************
 *
 ******************************************************************************/
class BulkInEndpointViaSTM32F4 {
private:
    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/
    InEndpointViaSTM32F4 &  m_inEndpoint;

public:
    constexpr BulkInEndpointViaSTM32F4(InEndpointViaSTM32F4 &p_inEndpoint) : m_inEndpoint(p_inEndpoint) {

    }

    ~BulkInEndpointViaSTM32F4() {

    }

    void enable(void) const {
        this->m_inEndpoint.enable(UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk);
    };

    void disable(void) const {
        this->m_inEndpoint.disable();
    };

    void write(const uint8_t * const p_data, const size_t p_length) const {
        this->m_inEndpoint.write(p_data, p_length);
    };
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8 */
