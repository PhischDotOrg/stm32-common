/*-
 * $Copyright$
-*/

#ifndef _DMA_CHANNEL_HPP_e94ddf65_7e91_4056_a4ce_6e2f83ff5102
#define _DMA_CHANNEL_HPP_e94ddf65_7e91_4056_a4ce_6e2f83ff5102

#include <stddef.h>
#include <stdint.h>

#if defined(DEBUG_BUILD) && defined(GMOCK_FOUND)
#include <gmock/gmock.h>
#endif /* defined(DEBUG_BUILD) && defined(GMOCK_FOUND) */

#if defined(STM32F4)
#undef STM32F4
#define STM32F4 STM32F4_B7E2C5A5_63D7_4D94_B530_1F0F73325305
#endif /* defined(STM32F4) */

/*******************************************************************************
 * 
 ******************************************************************************/
#if !defined(DMA_ENV)
#define DMA_ENV Undefined
#endif

/*******************************************************************************
 * 
 ******************************************************************************/
namespace dma {

typedef enum DmaImpl_e {
    GTest,
    STM32F4_842028AF_A814_45AC_8934_D0E3CC637D01,
    Undefined,
} DmaImpl_t;

} /* namespace dma */

/*******************************************************************************
 * 
 ******************************************************************************/
#include "dma/DmaChannelViaSTM32F4.hpp"

/*******************************************************************************
 * DMA Channel Type Definition
 ******************************************************************************/
namespace dma {

template<DmaImpl_t> struct DmaChannelChoice;

class DmaChannelMock;

template<> struct DmaChannelChoice<GTest> {
    typedef DmaChannelMock m_type;
};

template<> struct DmaChannelChoice<STM32F4_842028AF_A814_45AC_8934_D0E3CC637D01> {
    typedef DmaChannelViaSTM32F4T<> m_type;
};

template<> struct DmaChannelChoice<Undefined> {
    typedef void m_type;
};

struct DmaChannelT {
    typedef DmaChannelChoice<DMA_ENV>::m_type m_type;
};

typedef DmaChannelT::m_type DmaChannel;

} /* namespace dma */

#endif /* _DMA_CHANNEL_HPP_e94ddf65_7e91_4056_a4ce_6e2f83ff5102 */
