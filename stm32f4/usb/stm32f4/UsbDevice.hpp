/*-
 * $Copyright$
-*/

#ifndef _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d
#define _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d

#include "UsbCore.hpp"
#include <usb/UsbTypes.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbDeviceT> class InEndpointT;
template<typename UsbDeviceT> class OutEndpointT;
template<typename UsbCoreT> class UsbDeviceViaUsbCoreT;

class UsbRxFifoBase;
template<typename UsbDeviceT> class UsbRxFifoT;

/*******************************************************************************
 *
 ******************************************************************************/
class UsbDeviceBase {
public:
    typedef InEndpointT<UsbDeviceViaUsbCoreT<UsbCore> > InEndpoint;
    typedef OutEndpointT<UsbDeviceViaUsbCoreT<UsbCore> > OutEndpoint;

    enum DeviceSpeed_e {
        e_UsbFullSpeed = 0x3
    };

private:
    /*******************************************************************************
     * Private Variables
     ******************************************************************************/
    USB_OTG_DeviceTypeDef * const           m_usbDevice;
    const uint8_t                           m_maxConfigurations;
    const UsbDeviceDescriptor_t             m_deviceDescriptor;
    const UsbDeviceQualifierDescriptor_t    m_deviceQualifierDescriptor;
    const void * const                      m_configurationDescriptor;
    const ::usb::UsbStringDescriptors_t &   m_stringDescriptors;

protected:
    const DeviceSpeed_e             m_deviceSpeed;

    static const size_t             m_maxInEndpoints = 4;
    static const size_t             m_maxOutEndpoints = 4;

    InEndpoint *                    m_inEndpoints[m_maxInEndpoints];
    OutEndpoint *                   m_outEndpoints[m_maxOutEndpoints];

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
    void registerEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint);
    void registerEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint);

    void unregisterEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint);
    void unregisterEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint);

    void disableEndpointIrq(const InEndpoint &p_endpoint) const;
    void disableEndpointFifoIrq(const InEndpoint &p_endpoint) const;
    void enableEndpointIrq(const InEndpoint &p_endpoint) const;
    void enableEndpointFifoIrq(const InEndpoint &p_endpoint) const;

    void disableEndpointIrq(const OutEndpoint &p_endpoint) const;
    void enableEndpointIrq(const OutEndpoint &p_endpoint) const;

    void setAddress(const uint8_t p_address) const;
    void getDescriptor(const uint16_t p_descriptor, const size_t p_len) const;

    void setConfiguration(const uint8_t p_configuration) const;
    
    void getStatus(const uint8_t p_len) const;

protected:
            UsbDeviceBase(intptr_t p_deviceAddr, const void * const p_configurationDescriptor, const ::usb::UsbStringDescriptors_t &p_stringDescriptors, const DeviceSpeed_e p_speed);
    virtual ~UsbDeviceBase();

    void setupDeviceSpeed(const DeviceSpeed_e p_speed) const;
    
    void disableInterrupts(void) const;

    void disconnect(void) const;
    void connect(void) const;

    void handleEnumerationDone(void) const;
    void handleUsbReset(void) const;
    void handleStartOfFrame(void) const;
    void handleRxFIFO(const uint32_t p_rxStatus) const;
    void handleInEndpointIrq(void) const;
    void handleOutEndpointIrq(void) const;
    void handleDisconnectIrq(void) const;

    bool isSuspendableState(void) const;

    void disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;
    void enableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;

    virtual void setupTxFifos(void) const = 0;

private:
    void    getDeviceDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getDeviceQualifierDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getStringDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
    void    getConfigurationDescriptor(const uint8_t p_descriptorId, const size_t p_len) const;
};

/*******************************************************************************
 *
 ******************************************************************************/
template<typename UsbCoreT = UsbCore>
class UsbDeviceViaUsbCoreT : public UsbDeviceBase {
    friend class InEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> >;
    friend class OutEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> >;
    friend class UsbRxFifoT< UsbDeviceViaUsbCoreT<UsbCoreT> >;

public:
    UsbDeviceViaUsbCoreT(UsbCoreT &p_usbCore, const void * const p_configurationDescriptor, const ::usb::UsbStringDescriptors_t &p_stringDescriptors, const DeviceSpeed_e p_deviceSpeed = e_UsbFullSpeed);

    virtual ~UsbDeviceViaUsbCoreT();

    void setupTxFifo(const InEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> > &p_endpoint) const;
    void flushTxFifo(const InEndpointT< UsbDeviceViaUsbCoreT<UsbCoreT> > &p_endpoint) const;

    void initialize(void) const;

    void start(void) const;
    void stop(void) const;

    uint32_t handleIrq(const uint32_t p_irq) const;
    
protected:
    void handleUsbSuspend(void) const;
    void handleRxFIFO(void) const;
    void handleEnumerationDone(void) const;

    virtual void setupTxFifos(void) const;

private:
    UsbCoreT &  m_usbCore;

    constexpr intptr_t getBaseAddr(void) const {
        return m_usbCore.getBaseAddr();
    };
    
    typedef void (usb::stm32f4::UsbDeviceViaUsbCoreT<UsbCoreT>::*irq_handler_fn)() const;

    typedef struct irq_handler_s {
        uint32_t            m_irq;
        irq_handler_fn      m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];
};

/*******************************************************************************
 *
 ******************************************************************************/
typedef class UsbDeviceViaUsbCoreT<> UsbDevice;

    } /* namespace stm32f4 */
} /* namespace usb */

#include "UsbDevice.cpp"

#endif /* _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d */
