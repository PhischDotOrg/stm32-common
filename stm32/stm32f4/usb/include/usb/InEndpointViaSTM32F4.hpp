/*-
 * $Copyright$
 -*/

#ifndef _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8
#define _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8

#include <usb/UsbDeviceViaSTM32F4.hpp>
#include <stm32f4xx.h>

#include <usb/UsbInEndpoint.hpp>

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
class InEndpointViaSTM32F4 : public ::usb::UsbHwInEndpoint {
    friend class InEndpointViaSTM32F4Test;
private:
    void fillFifo(void) const;

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
     * IRQ Handler Functions
     ******************************************************************************/
    void    handleTxFifoEmpty(void) const;
    void    handleNakEffective(void) const;
    void    handleInTokenWhenTxFifoEmpty(void) const;
    void    handleTimeoutCondition(void) const;
    void    handleEndpointDisabled(void) const;
    void    handleTransferComplete(void) const;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    typedef enum class Interrupt_e : uint32_t {
        e_None                      = 0,
        e_TransferComplete          = USB_OTG_DIEPINT_XFRC,
        e_EndpointDisabled          = USB_OTG_DIEPINT_EPDISD,
        e_TimeoutCondition          = USB_OTG_DIEPINT_TOC,
        e_InTokenWhenTxFifoEmpty    = USB_OTG_DIEPINT_ITTXFE,
        // e_InTokenWithEndptMismatch  = USB_OTG_DIEPINT_INEPNM,
        e_InEndpointNakEffective    = USB_OTG_DIEPINT_INEPNE,
        e_TxFifoEmpty               = USB_OTG_DIEPINT_TXFE,
        // e_PacketDropped             = USB_OTG_PKTDRPSTS,
        // e_NakInput                  = USB_OTG_NAK,
        e_All                       = 0b1010001111111
    } Interrupt_t;

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
        /** @brief Interrupt to be handled. */
        Interrupt_t         m_irq;
        /** @brief Pointer to Interrupt Handler function. */
        irq_handler_fn      m_fn;
    } irq_handler_t;

    using irq_handler_table_t = std::array<const irq_handler_t, 4>;

    /***************************************************************************//**
     * \brief Table of IN Endpoint Interrupt Handlers.
     *
     * Table of interrupt handlers. Is handled in order from first to last, i.e.
     * functions listed earlier are handled before the functions listed later.
     *
     ******************************************************************************/
    static constexpr 
    irq_handler_table_t m_irq_handler { {
        { .m_irq = Interrupt_e::e_TransferComplete,         .m_fn = &InEndpointViaSTM32F4::handleTransferComplete },
        { .m_irq = Interrupt_e::e_TxFifoEmpty,              .m_fn = &InEndpointViaSTM32F4::handleTxFifoEmpty },
        // { .m_irq = Interrupt_e::e_InTokenWhenTxFifoEmpty,   .m_fn = &InEndpointViaSTM32F4::handleInTokenWhenTxFifoEmpty },
        { .m_irq = Interrupt_e::e_EndpointDisabled,         .m_fn = &InEndpointViaSTM32F4::handleEndpointDisabled }
    } };

    ::usb::UsbInEndpoint *  m_endpointCallback;

    /*******************************************************************************
     * Private Functions
     ******************************************************************************/
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
        return p_txLength ? (p_txLength + (p_packetSz - 1)) / p_packetSz : 1;
    }

protected:
    void initialize(const UsbDeviceViaSTM32F4::EndpointType_e &p_endpointType) const;

    virtual constexpr const UsbDeviceViaSTM32F4::EndpointType_e &getEndpointType(void) const = 0;

public:
    InEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber = 0);
    virtual ~InEndpointViaSTM32F4();

    /*******************************************************************************
     * Interface to UsbDeviceViaSTM32F4
     ******************************************************************************/
    void handleIrq(void) const;

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
    void enable(void) const override {
        initialize(getEndpointType());
    };
    void disable(void) const override;

    void ack(const size_t p_length) const override;
    void stall() const override;
    void nack() const override;

    void
    registerEndpointCallback(::usb::UsbInEndpoint &p_endpointCallback) override {
        assert(m_endpointCallback == nullptr);
        m_endpointCallback = &p_endpointCallback;
    }

    void
    unregisterEndpointCallback(void) override {
        m_endpointCallback = nullptr;
    }

    void setData(unsigned /* p_dtog */) const override { /* FIXME Implement this */ };
    bool getData(void) const override { /* FIXME Implement this */ return false; };
};

/*******************************************************************************
 *
 ******************************************************************************/
class CtrlInEndpointViaSTM32F4 : public InEndpointViaSTM32F4 {
    static constexpr UsbDeviceViaSTM32F4::EndpointType_e m_endpointType = UsbDeviceViaSTM32F4::EndpointType_e::e_Control;

protected:
    const UsbDeviceViaSTM32F4::EndpointType_e & getEndpointType(void) const override {
        return m_endpointType;
    }

public:
    CtrlInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords)
      : InEndpointViaSTM32F4(p_usbDevice, p_fifoSzInWords, 0) {

    }
};

/*******************************************************************************
 *
 ******************************************************************************/
class BulkInEndpointViaSTM32F4 : public InEndpointViaSTM32F4 {
    static constexpr UsbDeviceViaSTM32F4::EndpointType_e m_endpointType = UsbDeviceViaSTM32F4::EndpointType_e::e_Bulk;

protected:
    const UsbDeviceViaSTM32F4::EndpointType_e & getEndpointType(void) const override {
        return m_endpointType;
    }

public:
    BulkInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
      : InEndpointViaSTM32F4(p_usbDevice, p_fifoSzInWords, p_endpointNumber) {
        assert(p_endpointNumber != 0);
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
class IrqInEndpointViaSTM32F4 : public InEndpointViaSTM32F4 {
    static constexpr UsbDeviceViaSTM32F4::EndpointType_e m_endpointType = UsbDeviceViaSTM32F4::EndpointType_e::e_Interrupt;

protected:
    const UsbDeviceViaSTM32F4::EndpointType_e & getEndpointType(void) const override {
        return m_endpointType;
    }

public:
    IrqInEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber)
      : InEndpointViaSTM32F4(p_usbDevice, p_fifoSzInWords, p_endpointNumber) {
        assert(p_endpointNumber != 0);
    }
};

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/

#endif /* _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8 */
