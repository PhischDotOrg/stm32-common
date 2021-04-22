/*-
 * $Copyright$
-*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <exception>
#include <string>

#include <usb/UsbDeviceViaSTM32F4.hpp>

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/******************************************************************************/
class UsbDeviceViaSTM32F4Test : public ::testing::Test {
public:
    void SetUp() override {
    }

    void TearDown() override {
    }
};
/******************************************************************************/

TEST_F(UsbDeviceViaSTM32F4Test, CreateAndDelete) {
    /* Intentionally left blank */
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/
