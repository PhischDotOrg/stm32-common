/*-
 * $Copyright$
-*/

#ifndef _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d
#define _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d

#include <usb/UsbHwDevice.hpp>
#include <usb/UsbTypes.hpp>
#include <usb/UsbCoreViaSTM32F4.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
class InEndpointViaSTM32F4;
class CtrlOutEndpointViaSTM32F4;
class OutEndpointViaSTM32F4;

/*******************************************************************************
 *
 ******************************************************************************/
class UsbDeviceViaSTM32F4 : public UsbHwDevice {
public:
    typedef enum DataPID_e {
        e_Data0 = 0x0,
        e_Data1 = 0x2,
        e_Data2 = 0x1,
        e_MData = 0x3
    } DataPID_t;

    typedef enum EndpointType_e {
        e_Control       = EP_TYPE_CTRL,
        e_Isochronous   = EP_TYPE_ISOC,
        e_Bulk          = EP_TYPE_BULK,
        e_Interrupt     = EP_TYPE_ISOC
    } EndpointType_t;

private:
    /*******************************************************************************
     * Private Variables
     ******************************************************************************/
    UsbCoreViaSTM32F4 &  m_usbCore;

    typedef void (usb::stm32f4::UsbDeviceViaSTM32F4::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        UsbCoreViaSTM32F4::Interrupt_e  m_irq;
        irq_handler_fn                  m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    USB_OTG_DeviceTypeDef * const           m_usbDevice;

protected:
    static const size_t             m_maxInEndpoints = 4;
    static const size_t             m_maxOutEndpoints = 4;

    InEndpointViaSTM32F4 *          m_inEndpoints[m_maxInEndpoints];
    CtrlOutEndpointViaSTM32F4 *     m_ctrlOutEndpoint;
    OutEndpointViaSTM32F4 *         m_outEndpoints[m_maxOutEndpoints];
    union UsbDeviceStatus_u {
        uint16_t        m_status;
        struct {
            uint16_t    m_selfPowered   : 1;
            uint16_t    m_remoteWakeup  : 1;
            uint16_t    m_reserved      : 14;
        } m_bitfield;
    } __attribute__((packed));

    typedef union UsbDeviceStatus_u UsbDeviceStatus_t;

    UsbDeviceStatus_t               m_deviceStatus;
    
public:
    UsbDeviceViaSTM32F4(UsbCoreViaSTM32F4 &p_usbCore, const DeviceSpeed_e p_speed = DeviceSpeed_e::e_UsbFullSpeed);
    virtual ~UsbDeviceViaSTM32F4();

    void registerEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint);
    void unregisterEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint);

    void registerEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint);
    void unregisterEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint);

    void registerEndpoint(CtrlOutEndpointViaSTM32F4 &p_endpoint);
    void unregisterEndpoint(CtrlOutEndpointViaSTM32F4 &p_endpoint);

    void disableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void disableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const;

    void disableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const;

    void getStatus(const uint8_t p_len) const;

    void setupTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const;
    void flushTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const;

    void initialize(void) const;

    void start(void) const;
    void stop(void) const;

    uint32_t handleIrq(const uint32_t p_irq) const;
    
    intptr_t getBaseAddr(void) const {
        return m_usbCore.getBaseAddr();
    };

    DeviceSpeed_e getEnumeratedSpeed(void) const;

/*******************************************************************************
 * UsbHwDevice Interface
 ******************************************************************************/
    void setAddress(const uint8_t p_address) const;
   
#if defined(USB_DEBUG)
    void debugPrint(void) const;
#endif /* defined(USB_DEBUG) */

private:
    void setupDeviceSpeed(const DeviceSpeed_e p_speed) const;
    
    void disableInterrupts(void) const;

    void disconnect(void) const;
    void connect(void) const;

    void handleEnumerationDone(void) const;
    void handleUsbReset(void) const;
    void handleStartOfFrame(void) const;
    void handleRxFIFO(void) const;
    void handleInEndpointIrq(void) const;
    void handleOutEndpointIrq(void) const;
    void handleConnectorIdStatusChangeIrq(void) const;
    void handleEarlySuspendIrq(void) const;
    void handleUsbSuspendIrq(void) const;

    bool isSuspendableState(void) const;

    void disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;
    void enableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;

    virtual void setupTxFifos(void) const;

    void    getDeviceDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getDeviceQualifierDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getStringDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getConfigurationDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d */
