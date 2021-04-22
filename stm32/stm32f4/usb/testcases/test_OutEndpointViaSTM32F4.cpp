/*-
 * $Copyright$
-*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <exception>
#include <string>

#include <usb/OutEndpointViaSTM32F4.hpp>

/******************************************************************************/
class UsbOutEndpointMock : public ::usb::UsbOutEndpoint {
public:
    MOCK_METHOD(void, handleTransferComplete, ());
};
/******************************************************************************/

/******************************************************************************/
class UsbCoreViaSTM32F4Mock : public ::stm32::usb::UsbCoreViaSTM32F4 {
protected:
    uint32_t                        m_usbPowerCtrlHwMock {};
    static constexpr uint32_t       m_rxFifoSzInWords = 0x20;

public:
    UsbCoreViaSTM32F4Mock(USB_OTG_GlobalTypeDef &p_usbCoreHwMock)
      : UsbCoreViaSTM32F4(&p_usbCoreHwMock, reinterpret_cast<intptr_t>(&m_usbPowerCtrlHwMock), m_rxFifoSzInWords)
    {

    }

    UsbCoreViaSTM32F4Mock(const UsbCoreViaSTM32F4Mock &) = delete;
};
/******************************************************************************/

/******************************************************************************/
class UsbDeviceViaSTM32F4Mock : public ::stm32::usb::UsbDeviceViaSTM32F4 {
public:
    UsbDeviceViaSTM32F4Mock(::stm32::usb::UsbCoreViaSTM32F4 &p_usbCoreMock)
      : UsbDeviceViaSTM32F4(p_usbCoreMock)
    {

    }

    UsbDeviceViaSTM32F4Mock(const UsbDeviceViaSTM32F4Mock &) = delete;
};
/******************************************************************************/

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/******************************************************************************/
class OutEndpointViaSTM32F4Test : public ::testing::Test {
protected:
    USB_OTG_GlobalTypeDef       m_usbCoreHwMock {
        .GRSTCTL    = USB_OTG_GRSTCTL_AHBIDL
    };

    USB_OTG_OUTEndpointTypeDef  m_endpointHwMock {

    };

    UsbCoreViaSTM32F4Mock       m_usbCoreMock;
    UsbDeviceViaSTM32F4Mock     m_usbDeviceMock;
    UsbOutEndpointMock          m_usbOutEndpointMock;

    OutEndpointViaSTM32F4       m_outEndpt;

public:
    OutEndpointViaSTM32F4Test(void) : m_usbCoreMock(m_usbCoreHwMock), m_usbDeviceMock(m_usbCoreMock), m_outEndpt(m_usbDeviceMock, m_usbOutEndpointMock, 1) {

    }

    void SetUp() override {
    }

    void TearDown() override {
    }
};
/******************************************************************************/

TEST_F(OutEndpointViaSTM32F4Test, CreateAndDelete) {
    /* Intentionally left blank */
}

TEST_F(OutEndpointViaSTM32F4Test, dataReceivedDeviceCallback) {
    /* FIXME Re-design so that FIFO Data can be simulated and tested */
    m_outEndpt.dataReceivedDeviceCallback(4, UsbDeviceViaSTM32F4::DataPID_e::e_Data0);
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/
