/*-
 * $Copyright$
-*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <exception>
#include <string>

#include <usb/UsbCoreViaSTM32F4.hpp>

using ::testing::InSequence;

#if defined(__cplusplus)
extern "C" {
#endif  /* defined(__cplusplus) */

void
halt(const char * const p_file, unsigned p_line) {
    std::stringstream exceptionMsg;
    
    exceptionMsg << p_file << ":" << p_line;

    throw std::runtime_error(exceptionMsg.str());
}

#if defined(__cplusplus)
}; /* extern "C" */
#endif /* defined(__cplusplus) */

static bool
BitSet(uint32_t p_register, uint32_t p_mask) {
    return ((p_register & p_mask) == p_mask);
}

static bool
BitField(uint32_t p_register, uint32_t p_startBit, uint32_t p_endBit, uint32_t p_value) {
    uint32_t mask = 0;

    assert(p_endBit < sizeof(uint32_t) * 8);

    for (unsigned i = p_startBit; i <= p_endBit; i++) {
        mask |= (1 << i);
    }

    return ((p_register & mask) >> p_startBit) == p_value;
}

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/*******************************************************************************
 *
 ******************************************************************************/
class UsbCoreViaSTM32F4Test : public ::testing::Test {
protected:
    USB_OTG_GlobalTypeDef                           m_usbOtgRegisters;
    UsbCoreViaSTM32F4::PowerAndClockGatingControl_t m_usbPowerCtrlRegisters;   
    UsbCoreViaSTM32F4 *                             m_usbCore;
    const uint32_t                                  m_rxFifoSzInWords;

public:
    UsbCoreViaSTM32F4Test() : m_rxFifoSzInWords(256) {

    }

    void SetUp() override {
        m_usbCore = new UsbCoreViaSTM32F4(&m_usbOtgRegisters, reinterpret_cast<intptr_t>(&m_usbPowerCtrlRegisters), m_rxFifoSzInWords);

        /* AHB Idle, required by UsbCoreViaSTM32F4::performReset */
        m_usbOtgRegisters.GRSTCTL |= USB_OTG_GRSTCTL_AHBIDL;

        m_usbCore->setupMode(UsbCoreViaSTM32F4::DeviceMode_e::e_UsbDevice);

        /* Force Device Mode, required by UsbCoreViaSTM32F4::setupTxFifo */
        ASSERT_PRED2(BitSet, m_usbOtgRegisters.GUSBCFG, USB_OTG_GUSBCFG_FDMOD);
    }

    void TearDown() override {
        delete m_usbCore;
    }
};

/*******************************************************************************
 *
 ******************************************************************************/
TEST_F(UsbCoreViaSTM32F4Test, CreateAndDelete) {
    /* Intentionally left blank */
}

/*******************************************************************************
 *
 ******************************************************************************/
TEST_F(UsbCoreViaSTM32F4Test, InvalidRxFifoSize) {
    EXPECT_EXIT(UsbCoreViaSTM32F4(nullptr, 0, 15), ::testing::KilledBySignal(SIGABRT), "Assertion failed");
    EXPECT_EXIT(UsbCoreViaSTM32F4(nullptr, 0, 257), ::testing::KilledBySignal(SIGABRT), "Assertion failed");
}

/*******************************************************************************
 *
 ******************************************************************************/
TEST_F(UsbCoreViaSTM32F4Test, DISABLED_SetupTxFifoCtrlEndpoint) {
    this->m_usbCore->setupTxFifo(0, 0x20);

    EXPECT_PRED4(BitField, this->m_usbOtgRegisters.DIEPTXF0_HNPTXFSIZ, 16, 31, 0x20);
    EXPECT_PRED4(BitField, this->m_usbOtgRegisters.DIEPTXF0_HNPTXFSIZ,  0, 15, m_rxFifoSzInWords * sizeof(uint32_t));
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/
