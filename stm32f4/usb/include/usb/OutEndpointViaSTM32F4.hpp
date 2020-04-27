/*-
 * $Copyright$
-*/

#ifndef _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9
#define _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9

#include <stm32f4xx.h>

#include <usb/UsbHwOutEndpoint.hpp>

#include <usb/UsbDeviceViaSTM32F4.hpp>

namespace usb {    
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
class OutEndpointViaSTM32F4 : public UsbHwOutEndpoint {
private:
    UsbDeviceViaSTM32F4 &               m_usbDevice;
    const unsigned                      m_endpointNumber;
    USB_OTG_OUTEndpointTypeDef * const  m_endpoint;
    volatile uint32_t * const           m_fifoAddr;

    /*******************************************************************************
     * Typedefs and static buffer for IRQ Handler
     ******************************************************************************/
    typedef void (usb::stm32f4::OutEndpointViaSTM32F4 ::*irq_handler_fn)();

    typedef struct irq_handler_s {
        uint32_t m_irq;
        irq_handler_fn m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /*******************************************************************************
     * Private Functions
     ******************************************************************************/
    void                setNack(const bool p_nack) const;
    void                handleSetupDoneIrq(void);
    void                handleTransferComplete();
    void                handleDeviceRequest(void) const;

public:
    OutEndpointViaSTM32F4(UsbDeviceViaSTM32F4 &p_usbDevice, const unsigned p_endpointNumber = 0);
    ~OutEndpointViaSTM32F4();

    void            reset(void) const;

    void            disable(void) const;
    void            enable(void) const;

    void            handleIrq(void);

    void            handleRxData(const size_t p_numBytes, const typename UsbDeviceViaSTM32F4::DataPID_e &p_dataPID);
    void            handleSetupData(const size_t p_numBytes);
    void            handleSetupComplete(void);

    void            handleOutTransferComplete(void);

    unsigned getEndpointNumber(void) const {
        return this->m_endpointNumber;
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _OUTENDPOINT_HPP_669fcf90_b666_4583_97b7_e50a9d1447d9 */
