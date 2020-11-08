/*-
 * $Copyright$
-*/

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <exception>
#include <string>

#include <usb/InEndpointViaSTM32F4.hpp>

/******************************************************************************/
namespace stm32 {
    namespace usb {
/******************************************************************************/

/******************************************************************************/
class InEndpointViaSTM32F4Test : public ::testing::Test {
protected:
    static unsigned
    getNumPackets(const size_t p_txLength, const size_t p_packetSz) {
        return InEndpointViaSTM32F4::getNumPackets(p_txLength, p_packetSz);
    }
    
public:
    InEndpointViaSTM32F4Test() {
    }

    void SetUp() override {
    }

    void TearDown() override {
    }
};
/******************************************************************************/

TEST_F(InEndpointViaSTM32F4Test, CreateAndDelete) {
    /* Intentionally left blank */
}

TEST_F(InEndpointViaSTM32F4Test, getNumPackets3) {
    static const unsigned packetSz = 3;

    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 1, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 2, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 3, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 4, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 5, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 6, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 7, packetSz));
}

TEST_F(InEndpointViaSTM32F4Test, getNumPackets8) {
    static const unsigned packetSz = 8;

    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 1, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 7, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 8, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 9, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 15, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 16, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 17, packetSz));
}

TEST_F(InEndpointViaSTM32F4Test, getNumPackets64) {
    static const unsigned packetSz = 64;

    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 1, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 1, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 32, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 33, packetSz));
    EXPECT_EQ(1, getNumPackets(/* p_txLength */ 63, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 64, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 65, packetSz));
    EXPECT_EQ(2, getNumPackets(/* p_txLength */ 127, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 128, packetSz));
    EXPECT_EQ(3, getNumPackets(/* p_txLength */ 129, packetSz));
}

/******************************************************************************/
    } /* namespace usb */
} /* namespace stm32 */
/******************************************************************************/
