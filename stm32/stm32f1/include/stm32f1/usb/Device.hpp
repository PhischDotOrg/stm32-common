/*-
 * $Copyright$
 */

#ifndef __STM32F1_USB_DEVICE_HPP_F9DF4364_64FB_4823_BF3A_5821E4823740
#define __STM32F1_USB_DEVICE_HPP_F9DF4364_64FB_4823_BF3A_5821E4823740

#include <usb/UsbHwDevice.hpp>
#include <usb/UsbTypes.hpp>
#include "stm32f1/usb/Peripheral.hpp"

#include <stm32f4xx.h>

#include <cstddef>
#include <array>

#include "f1usb/src/usbutils.hh"

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

class InEndpoint;
class OutEndpoint;
class CtrlOutEndpoint;

struct EndpointBufferDescriptor_s {
    uint16_t    m_txAddr;
    uint16_t    m_padding_0;
    uint16_t    m_txCount;
    uint16_t    m_padding_1;
    uint16_t    m_rxAddr;
    uint16_t    m_padding_2;
    uint16_t    m_rxCount;
    uint16_t    m_padding_3;
};
static_assert(sizeof(UsbMem) == 4);
static_assert(sizeof(EndpointBufferDescriptor_s) == 16);

class Device : public ::usb::UsbHwDevice {
    static constexpr unsigned   m_maxEndpoints = 8;
    static constexpr unsigned   m_maxOutEndpoints = 4;
    static constexpr unsigned   m_maxInEndpoints = 4;

    static_assert((m_maxInEndpoints + m_maxOutEndpoints) <= m_maxEndpoints);

    Peripheral &                        m_usbPeripheral;
    InEndpoint *                        m_inEndpoints[m_maxInEndpoints];
    OutEndpoint *                       m_outEndpoints[m_maxOutEndpoints];
    CtrlOutEndpoint *                   m_ctrlOutEndpoint;
    alignas(8) static struct EndpointBufferDescriptor_s m_bufferDescriptorTable[m_maxEndpoints] USB_MEM;

public:
    typedef struct EndpointBufferDescriptor_s EndptBufDescr_t;

    Device(Peripheral &p_usbPeripheral) :
      ::usb::UsbHwDevice(DeviceSpeed_e::e_UsbFullSpeed),
      m_usbPeripheral(p_usbPeripheral)
    {
        m_usbPeripheral.registerDevice(*this);
    }

    virtual ~Device() {
        m_usbPeripheral.unregisterDevice();
    }

    void setAddress(const uint8_t p_address) const override {
        m_usbPeripheral.setAddress(p_address);
    }

    void
    registerCtrlEndpoint(CtrlOutEndpoint &p_endpoint) {
        assert(this->m_ctrlOutEndpoint == nullptr);

        this->m_ctrlOutEndpoint = &p_endpoint;
    }

    void
    registerInEndpoint(const unsigned p_endpointNumber, InEndpoint &p_endpoint) {
        assert(p_endpointNumber < this->m_maxInEndpoints);
        assert(this->m_inEndpoints[p_endpointNumber] == nullptr);

        this->m_inEndpoints[p_endpointNumber] = &p_endpoint;
    }

    void
    unregisterInEndpoint(const unsigned p_endpointNumber) {
        assert(p_endpointNumber < this->m_maxInEndpoints);
        assert(this->m_inEndpoints[p_endpointNumber] != nullptr);

        this->m_inEndpoints[p_endpointNumber] = nullptr;
    }

    void
    registerOutEndpoint(const unsigned p_endpointNumber, OutEndpoint &p_endpoint) {
        assert(p_endpointNumber < this->m_maxOutEndpoints);
        assert(this->m_outEndpoints[p_endpointNumber] == nullptr);

        this->m_outEndpoints[p_endpointNumber] = &p_endpoint;
    }

    void
    unregisterOutEndpoint(const unsigned p_endpointNumber) {
        assert(p_endpointNumber < this->m_maxOutEndpoints);
        assert(this->m_outEndpoints[p_endpointNumber] != nullptr);

        this->m_outEndpoints[p_endpointNumber] = nullptr;
    }

    static constexpr
    EndptBufDescr_t &    
    getEndptBufferDescr(unsigned p_endptPhysIdx) {
        assert(p_endptPhysIdx < m_maxEndpoints);

        return m_bufferDescriptorTable[p_endptPhysIdx];
    }

    void start(void) const {
        this->m_usbPeripheral.start();
    }

    void stop(void) const {
        this->m_usbPeripheral.stop();
    }

    void reset(void) const;

    uintptr_t
    getBaseAddr(void) const {
        return m_usbPeripheral.getBaseAddr();
    }

    static constexpr uint16_t
    mapHostToPeripheral(uintptr_t p_hostAddr) {
        return Peripheral::mapHostToPeripheral(p_hostAddr);
    }

    void handleEndpointIrq(unsigned p_endpointNo, unsigned p_direction) const;
};

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* __STM32F1_USB_DEVICE_HPP_F9DF4364_64FB_4823_BF3A_5821E4823740 */
