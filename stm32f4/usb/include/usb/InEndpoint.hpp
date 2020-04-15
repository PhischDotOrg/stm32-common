/*-
 * $Copyright$
 -*/

#ifndef _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8
#define _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8

#include "UsbDevice.hpp"

namespace usb {
namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT = UsbDevice>
class InEndpointT {
private:
    UsbDeviceT &                        m_usbDevice;
    const unsigned                      m_endpointNumber;
    USB_OTG_INEndpointTypeDef * const   m_endpoint;
    volatile uint32_t * const           m_txFifoAddr;
    const size_t                        m_fifoSzInWords;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    typedef void (usb::stm32f4::InEndpointT<UsbDeviceT> ::*irq_handler_fn)();

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

protected:

public:
    InEndpointT(UsbDeviceT &p_usbDevice, const size_t p_fifoSzInWords, const unsigned p_endpointNumber = 0);
    ~InEndpointT();

    void disableIrq(void) const;
    void enableIrq(void) const;

    void setupTxFifoNumber(const unsigned p_fifoNumber) const;
    void reset(void) const;

    void disable(void) const;
    void enable(void) const;

    void handleIrq(void);

    void setPacketSize(const unsigned p_packetSize) const;

    void write(const uint8_t * const p_data, const size_t p_dataLength, const size_t p_txLength);
    void writeString(const ::usb::UsbStringDescriptor &p_string, const size_t p_len);

    unsigned getEndpointNumber(void) const {
        return this->m_endpointNumber;
    }

    size_t getFifoSzInWords(void) const {
        return this->m_fifoSzInWords;
    }

};

/*******************************************************************************
 *
 ******************************************************************************/
typedef InEndpointT<> InEndpoint;

} /* namespace stm32f4 */
} /* namespace usb */

#include <usb/InEndpoint.cpp>

#endif /* _INENDPOINT_HPP_34f149e0_9f89_489b_a2c0_95b3363fe4a8 */
