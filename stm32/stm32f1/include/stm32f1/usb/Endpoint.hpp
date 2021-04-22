/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_ENDPOINT_HPP_1951DDFD_9FCF_4180_B4F5_6D1B655865FF
#define __STM32F1_USB_ENDPOINT_HPP_1951DDFD_9FCF_4180_B4F5_6D1B655865FF

#include "stm32f1/usb/Endpoint.hpp"
#include "stm32f1/usb/Device.hpp"

#include "f1usb/src/usbutils.hh"

#include <utility>
#include <cassert>
#include <cstddef>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

class Device;

class Endpoint {
protected:
    /**
     * @brief Reference to the STM32F1 USB Device Driver Object.
     *
     */
    Device &                    m_usbDevice;
    volatile USBEndpt_TypeDef & m_hwEndpt;

protected:
    /**
     * @brief Endpoint Number (without Direction Bit).
     *
     * Endpoint Number, encoded without the Direction Bit. Can be set to a range
     * between 0 and ::stm32::f1::usb::Device::m_maxEndpoints as it's
     * used to refer to the Endpoint Number within the STM32F1 USB Device Hardware.
     */
    const unsigned      m_endpointNumber;

    const std::pair<UsbMem * const, size_t> m_buffer;

    struct EndpointBufferDescriptor_s & m_endptBufferDescr;

    typedef enum class Interrupt_e : uint16_t {
        e_None              = 0,
        e_CorrectTransferRx = USB_EP0R_CTR_RX,
        e_SetupComplete     = USB_EP0R_SETUP,
        e_CorrectTransferTx = USB_EP0R_CTR_TX,
        e_All               = (USB_EP0R_CTR_RX | USB_EP0R_CTR_TX)
    } Interrupt_t;

    void    clearInterrupt(Interrupt_t p_irq) const;

    void    reset(void) const;

    void    rxEnable(void) const;
    void    txEnable(void) const;
    void    disable(void) const;

    typedef enum class EndpointType_e : uint8_t {
        e_Bulk          = 0b00,
        e_Control       = 0b01,
        e_Isochronous   = 0b10,
        e_Interrupt     = 0b11
    } EndpointType_t;
    void    setEndpointType(EndpointType_t p_endpointType) const;

    enum class Status_e : uint8_t {
        e_Disabled      = 0b00,
        e_Stall         = 0b01,
        e_Nak           = 0b10,
        e_Valid         = 0b11
    };
    using EndpointStatus_t = enum Status_e;

    void setRxStatus(EndpointStatus_t p_rxStatus) const;
    void setTxStatus(EndpointStatus_t p_rxStatus) const;

    bool getCtrRx(void) const;
    void clrCtrRx(void) const;

    bool getCtrTx(void) const;
    void clrCtrTx(void) const;

    void setAddress(const unsigned p_endpointNumber) const;

    enum class DataToggleRx_e : uint8_t {
        e_Data0 = 0,
        e_Data1 = 1
    };

    enum class DataToggleTx_e : uint8_t {
        e_Data0 = 0,
        e_Data1 = 1
    };

    size_t enqueueInPacket(const size_t p_lengt) const;

    void setDataToggleRx(DataToggleRx_e p_dataToggle) const;
    DataToggleRx_e getDataToggleRx(void) const {
        return (m_hwEndpt.EPxR & USB_EP_DTOG_RX_Msk) ? DataToggleRx_e::e_Data1 : DataToggleRx_e::e_Data0;
    }

    void setDataToggleTx(DataToggleTx_e p_dataToggle) const;
    DataToggleTx_e getDataToggleTx(void) const {
        return (m_hwEndpt.EPxR & USB_EP_DTOG_TX_Msk) ? DataToggleTx_e::e_Data1 : DataToggleTx_e::e_Data0;
    }

    void setEPnR(uint16_t p_mask, uint16_t p_data, uint16_t p_old) const;
    void setEPnR(uint16_t p_mask, uint16_t p_data) const;

public:
    using Buffer_t = UsbMem *;

    Endpoint(Endpoint &) = delete;

    Endpoint(Device &p_usbDevice, const unsigned p_endpointNumber, Endpoint::Buffer_t &p_buffer, const size_t p_length)
      : m_usbDevice(p_usbDevice),
        m_hwEndpt(m_usbDevice.getHwEndpt(p_endpointNumber)),
        m_endpointNumber(p_endpointNumber),
        m_buffer(std::make_pair(p_buffer, p_length)),
        m_endptBufferDescr(m_usbDevice.getEndptBufferDescr(m_endpointNumber))
    {
    }
};

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_ENDPOINT_HPP_1951DDFD_9FCF_4180_B4F5_6D1B655865FF */
