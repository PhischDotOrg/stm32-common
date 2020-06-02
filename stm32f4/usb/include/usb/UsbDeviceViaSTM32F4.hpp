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

/***************************************************************************//**
 * @brief Driver for the STM32F4 USB On-the-Go (OTG) Core in USB Device Mode.
 * 
 * Class that encapsulates the operation of the STM32F4's USB On-the-Go (OTG)
 * Core in USB Device Mode. 
 ******************************************************************************/
class UsbDeviceViaSTM32F4 : public UsbHwDevice {
public:
    /**
     * @brief Typedef for USB Packet ID (PID).
     * 
     * Type for the USB Packet ID of a received packet.
     * 
     * \warning Please note that the values are according to the definition of
     *   the \c DPID Bits in the \c GRXSTSP register -- not according to the USB
     *   standard.
     * 
     * \see UsbDeviceViaSTM32F4::handleRxFIFO
     * \see OutEndpointViaSTM32F4::dataReceivedDeviceCallback
     */
    typedef enum DataPID_e {
        e_Data0 = 0x0,
        e_Data1 = 0x2,
        e_Data2 = 0x1,
        e_MData = 0x3
    } DataPID_t;

    /**
     * @brief Typedef for USB Endpoint Type.
     * 
     * Type to classify an endpoint's type.
     * 
     * Maps the \c EPTYP Bits in Register \c DIEPCTL (for IN Endpoints) as well
     * as the \c EPTYP Bits in the \c DOEPCTL Register to a C++ type.
     * 
     * \see OutEndpointViaSTM32F4::setup
     * \see InEndpointViaSTM32F4::setupEndpointType
     */
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
    /** @brief Reference to the underlying UsbCoreViaSTM32F4 object. */
    UsbCoreViaSTM32F4 &  m_usbCore;

    /**
     * \brief Private Typedef for the USB Device IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef void (usb::stm32f4::UsbDeviceViaSTM32F4::*irq_handler_fn)() const;

    /**
     * @brief Private Data Type to construct the Table of IRQ Handlers.
     * 
     * \see #m_irq_handler
     */
    typedef struct irq_handler_s {
        /** @brief Type of Interrupt. */
        UsbCoreViaSTM32F4::Interrupt_e  m_irq;
        /** @brief Reference to IRQ handler member function. */
        irq_handler_fn                  m_fn;
    } irq_handler_t;

    static const irq_handler_t m_irq_handler[];

    /** @brief Reference to the USB Core's Device-mode Registers. */
    USB_OTG_DeviceTypeDef * const           m_usbDevice;

protected:
    /**
     * @brief Max. number of IN endpoints supported by this Hardware.
     * 
     * The STM32F4 USB Hardware supports up to four IN endpoints, including
     * the default control IN endpoint.
     * 
     * From a user perspective there are therefore three IN endpoints that
     * can be used freely.
     */
    static const size_t             m_maxInEndpoints = 4;

    /**
     * @brief Max. number of OUT endpoints supported by this Hardware.
     * 
     * The STM32F4 USB Hardware supports up to four OUT endpoints, including
     * the default control OUT endpoint.
     * 
     * From a user perspective there are therefore three OUT endpoints that
     * can be used freely.
     */
    static const size_t             m_maxOutEndpoints = 4;

    /**
     * @brief References to the IN Endpoint Handlers.
     * 
     * Pointers to InEndpointViaSTM32F4 objects for the IN Endpoints. The offset
     * within the array corresponds to the endpoint's number.
     * 
     * \see registerInEndpoint
     * \see unregisterEndpoint
     */
    InEndpointViaSTM32F4 *          m_inEndpoints[m_maxInEndpoints];

    /**
     * @brief Reference to the Control OUT Endpoint Handler.
     * 
     * The Control OUT Endpoint, i.e. OUT Endpoint Zero, is of a different type
     * because it can handle SETUP packets.
     * 
     * \see handleRxFIFO
     */
    CtrlOutEndpointViaSTM32F4 *     m_ctrlOutEndpoint;

    /**
     * @brief References to the OUT Endpoint Handlers.
     * 
     * Pointers to InEndpointViaSTM32F4 objects for the OUT Endpoints. The offset
     * within the array corresponds to the endpoint's number.
     * 
     * \see registerOutEndpoint
     * \see unregisterEndpoint
     */
    OutEndpointViaSTM32F4 *         m_outEndpoints[m_maxOutEndpoints];
    
public:
    UsbDeviceViaSTM32F4(UsbCoreViaSTM32F4 &p_usbCore, const DeviceSpeed_e p_deviceSpeed = DeviceSpeed_e::e_UsbFullSpeed);
    virtual ~UsbDeviceViaSTM32F4();

    void registerInEndpoint(const unsigned p_endpointNumber, InEndpointViaSTM32F4 &p_endpoint);
    void unregisterInEndpoint(const unsigned p_endpointNumber);

    void registerOutEndpoint(const unsigned p_endpointNumber, OutEndpointViaSTM32F4 &p_endpoint);
    void unregisterOutEndpoint(const unsigned p_endpointNumber);

    void registerCtrlEndpoint(CtrlOutEndpointViaSTM32F4 &p_endpoint);
    void unregisterCtrlEndpoint(void);

    void disableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void disableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointIrq(const InEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointFifoIrq(const InEndpointViaSTM32F4 &p_endpoint) const;

    void disableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const;
    void enableEndpointIrq(const OutEndpointViaSTM32F4 &p_endpoint) const;

    void setupTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const;
    void flushTxFifo(const InEndpointViaSTM32F4 &p_endpoint) const;

    void start(void) const;
    void stop(void) const;

    uint32_t handleIrq(const uint32_t p_irq) const;

    /**
     * @brief Get the Base Address of the USB Core's Registers.
     * 
     * Used by the Endpoint Handler classes to figure out the Base Address of the Endpoint-specific registers.
     * 
     * \see InEndpointViaSTM32F4::InEndpointViaSTM32F4
     * \see OutEndpointViaSTM32F4::OutEndpointViaSTM32F4
     *
     * @return constexpr intptr_t Base Address of the USB Core Registers.
     * 
     */
    constexpr intptr_t getBaseAddr(void) const {
        return m_usbCore.getBaseAddr();
    };

    DeviceSpeed_e getEnumeratedSpeed(void) const;

/*******************************************************************************
 * UsbHwDevice Interface
 ******************************************************************************/
    void setAddress(const uint8_t p_address) const override;

private:
    void setupDeviceSpeed(const DeviceSpeed_e p_speed) const;
    
    void disableInterrupts(void) const;

#if 0
    void disconnect(void) const;
    void connect(void) const;
#endif

    void handleEnumerationDone(void) const;
    void handleUsbReset(void) const;
    // void handleStartOfFrame(void) const;
    void handleRxFIFO(void) const;
    void handleInEndpointIrq(void) const;
    void handleOutEndpointIrq(void) const;
#if 0
    void handleConnectorIdStatusChangeIrq(void) const;
    void handleEarlySuspendIrq(void) const;
#endif
    void handleUsbSuspendIrq(void) const;

    bool isSuspendableState(void) const;

    void disableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;
    void enableEndpointIrq(const unsigned &p_endpointNumber, bool p_isOutEndpoint) const;

    void setupTxFifos(void) const;

    void initialize(void) const;
};

/*******************************************************************************
 *
 ******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#endif /* _USBDEVICE_HPP_32f4ccff_87e2_4eac_87e7_6297472e365d */
