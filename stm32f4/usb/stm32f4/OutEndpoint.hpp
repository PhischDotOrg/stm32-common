/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9
#define _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9

#include "UsbDevice.hpp"

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT = UsbDevice>
class OutEndpointT {
private:
    UsbDeviceT &                        m_usbDevice;
    const unsigned                      m_endpointNumber;
    USB_OTG_OUTEndpointTypeDef * const  m_endpoint;
    volatile uint32_t * const           m_rxFifoAddr;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    typedef void (usb::stm32f4::OutEndpointT<UsbDeviceT> ::*irq_handler_fn)();

    typedef struct irq_handler_s {
        uint32_t m_irq;
        irq_handler_fn m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /*******************************************************************************
     * Typedefs and static buffer for Setup Packet
     ******************************************************************************/
    struct SetupPacket_s {
        uint8_t m_bmRequestType;
        uint8_t m_bRequest;
        uint16_t m_wValue;
        uint16_t m_wIndex;
        uint16_t m_wLength;
    }__attribute__((packed));

    typedef struct SetupPacket_s SetupPacket_t;

    typedef enum UsbRecipient_e {
        e_Device        = 0x00,
        e_Interface     = 0x01,
        e_Endpoint      = 0x02,
        e_Other         = 0x03
    } UsbRecipient_t;

    typedef enum UsbRequest_e {
        e_GetStatus         = 0x00,
        e_ClearFeature      = 0x01,
        e_SetFeature        = 0x03,
        e_SetAddress        = 0x05,
        e_GetDescriptor     = 0x06,
        e_SetDescriptor     = 0x07,
        e_GetConfiguration  = 0x08,
        e_SetConfiguration  = 0x09,
        e_GetInterface      = 0x0A,
        e_SetInterface      = 0x0B,
        e_SyncFrame         = 0x0C
    } UsbRequest_t;

    typedef union RxBuffer_u {
        uint32_t        m_rxBuffer32[2];
        uint8_t         m_rxBuffer8[sizeof(m_rxBuffer32) / sizeof(m_rxBuffer32[0])];
        SetupPacket_t   m_setupPacket;
    } RxBuffer_t;
    RxBuffer_t      m_rxBuffer;
    unsigned        m_rxBufferCurPos;

    /*******************************************************************************
     * Private Functions
     ******************************************************************************/
    void                setNack(const bool p_nack) const;
    void                handleSetupDone(void);
    void                handleTransferComplete();
    void                handleDeviceRequest(void) const;

public:
    OutEndpointT(UsbDeviceT &p_usbDevice, const unsigned p_endpointNumber = 0);
    ~OutEndpointT();

    unsigned        getEndpointNumber(void) const { return this->m_endpointNumber; }

    void            reset(void) const;

    void            disable(void) const;
    void            enable(void) const;

    void            handleIrq(void);

    void            enableSetupPackets(const unsigned p_numPackets) const;

    void            handleRxData(const size_t p_numBytes);

#if 0
    void disableIrq(void) const {
        this->m_usbDevice.disableEndpointIrq(*this);
    }

    void enableIrq(void) const {
        this->m_usbDevice.enableEndpointIrq(*this);
    }
#endif
};

/*******************************************************************************
 *
 ******************************************************************************/
typedef OutEndpointT<> OutEndpoint;

    } /* namespace stm32f4 */
} /* namespace usb */

#include "OutEndpoint.cpp"

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
