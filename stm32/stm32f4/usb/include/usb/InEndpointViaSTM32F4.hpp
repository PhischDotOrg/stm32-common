/*-
 * $Copyright$
 -*/

#ifndef _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8
#define _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8

#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <stm32f4xx.h>

#include <cassert>
#include <cstddef>

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

class UsbDeviceViaSTM32F4;
class BulkInEndpointViaSTM32F4;
class CtrlInEndpointViaSTM32F4;
class IrqInEndpointViaSTM32F4;

/***************************************************************************//**
 * \brief IN Endpoint driver for STM32F4.
 * 
 * This class handles all IN Endpoint specific aspects of the STM32F4 USB Core.
 * 
 * It implements the interface to the device-independent USB  layer.
 * 
 * It also exposes an Interface to ::usb::stm32f4::UsbDeviceViaSTM32F4 to handle
 * callbacks from the USB Device Class.
 ******************************************************************************/
class InEndpointViaSTM32F4 {
    friend class InEndpointViaSTM32F4Test;
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

    /**
     * @brief Pointer to the ST-provided structure to access the Hardware registers.
     */
    USB_OTG_INEndpointTypeDef * const   m_endpoint;

    /**
     * @brief Pointer to the IN endpoint's Tx FIFO.
     * 
     * This is used by #txString and #txData to write Data into the Hardware Tx FIFO.
     */
    volatile uint32_t * const           m_fifoAddr;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    /**
     * \brief Private Typedef for the Endpoint IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef void (stm32::usb::InEndpointViaSTM32F4::*irq_handler_fn)(void) const;

    /**
     * @brief Typedef for IRQ Handler.
     */
    typedef struct irq_handler_s {
        uint32_t m_irq;
        irq_handler_fn m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /*******************************************************************************
     * Private Functions
     ******************************************************************************/
    void    setNack(const bool p_nack) const;

    void    startTx(size_t p_numBytes);

    void    txData(const uint8_t *p_data, const size_t p_len);

    void    handleTxFifoEmpty(void) const;
    void    handleNakEffective(void) const;
    void    handleInTokenWhenTxFifoEmpty(void) const;
    void    handleTimeoutCondition(void) const;
    void    handleEndpointDisabled(void) const;
    void    handleTransferComplete(void) const;

    void    reset(void) const;

    void    disableIrq(void) const;
    void    enableIrq(void) const;

    void
    disableFifoIrq(void) const {
        this->m_usbDevice.disableEndpointFifoIrq(*this);
    }

    void
    enableFifoIrq(void) const {
        this->m_usbDevice.enableEndpointFifoIrq(*this);
    }

    void    setupEndpointType(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const;

    unsigned    getPacketSize(void) const;

    static constexpr unsigned
    getNumPackets(const size_t p_txLength, const size_t p_packetSz) {
        return (1 + (p_txLength / p_packetSz));
    }

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
    InEndpointViaSTM32F4    m_inEndpoint;

public:
    CtrlInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords)
      : m_inEndpoint(p_usbDevice, p_fifoSzInWords, 0) {

    }

    void write(const uint8_t * const p_data, const size_t p_length) {
        this->m_inEndpoint.write(p_data, p_length);
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
    InEndpointViaSTM32F4    m_inEndpoint;

public:
    BulkInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
      : m_inEndpoint(p_usbDevice, p_fifoSzInWords, p_endpointNumber) {
        assert(p_endpointNumber != 0);
    }

    ~BulkInEndpointViaSTM32F4() {

    }

    void enable(void) const {
        this->m_inEndpoint.enable(UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk);
    };

    void disable(void) const {
        this->m_inEndpoint.disable();
    };

    void write(const uint8_t * const p_data, const size_t p_length) {
        this->m_inEndpoint.write(p_data, p_length);
    };
};

/*******************************************************************************
 *
 ******************************************************************************/
class IrqInEndpointViaSTM32F4 {
private:
    /*******************************************************************************
     * Private Class Attributes
     ******************************************************************************/
    InEndpointViaSTM32F4    m_inEndpoint;

public:
    IrqInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
      : m_inEndpoint(p_usbDevice, p_fifoSzInWords, p_endpointNumber) {
        assert(p_endpointNumber != 0);
    }

    void enable(void) const {
        this->m_inEndpoint.enable(UsbDeviceViaSTM32F4::EndpointType_e::e_Interrupt);
    };

    void disable(void) const {
        this->m_inEndpoint.disable();
    };

    void write(const uint8_t * const p_data, const size_t p_length) {
        this->m_inEndpoint.write(p_data, p_length);
    };
};

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8 */
