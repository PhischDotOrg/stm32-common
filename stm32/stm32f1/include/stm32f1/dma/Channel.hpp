/*-
 * $Copyright$
-*/
#ifndef _DMA_CHANNEL_STM32_HPP_B6BD9823_B66D_4724_9A96_965E03319555
#define _DMA_CHANNEL_STM32_HPP_B6BD9823_B66D_4724_9A96_965E03319555

#include <stdint.h>
#include <stddef.h>

#include <stm32/dma/Types.hpp>
#include <stm32f1/dma/Stream.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
/*****************************************************************************/

/*****************************************************************************/
class DmaChannel {
    DMA_Channel_TypeDef &   m_channel;

protected:
    DmaChannel(DMA_Channel_TypeDef *p_channel) : m_channel(*p_channel) {

    }

public:
    static constexpr
    intptr_t
    getDmaChannelAddress(intptr_t p_dmaEngine, unsigned p_channel) {
        return (p_dmaEngine + 0x1c + p_channel * 20);
    }

    void
    handleIrq(void) const {
        (void) m_channel;
    }
};
/*****************************************************************************/

/*****************************************************************************/
template<
    typename NvicT,
    typename DmaEngineT,
    unsigned nChannelNo
>
class DmaChannelT : public EngineT< DmaChannel::getDmaChannelAddress(DmaEngineT::m_engineType, nChannelNo) >, public DmaChannel {
    static_assert(nChannelNo >= 0);
    static_assert((DmaEngineT::m_engineType == DMA1_BASE) && (nChannelNo <= 7));

    const NvicT &   m_nvic;

public:
    DmaChannelT(const NvicT &p_nvic, const DmaEngineT & /* p_dmaEngine */)
      : DmaChannel(reinterpret_cast<DMA_Channel_TypeDef *>(DmaChannel::getDmaChannelAddress(DmaEngineT::m_engineType, nChannelNo))), m_nvic(p_nvic) {
        m_nvic.enableIrq(* static_cast< EngineT< DmaChannel::getDmaChannelAddress(DmaEngineT::m_engineType, nChannelNo) > *>(this));
    }

    ~DmaChannelT() {
        m_nvic.disableIrq(* static_cast< EngineT< DmaChannel::getDmaChannelAddress(DmaEngineT::m_engineType, nChannelNo) > *>(this));
    }

    void setup(const dma::DmaDirection_t /* p_direction */, const size_t /* p_length */) const {
        // this->m_stream.disable();
        // this->m_stream.setup(this->m_channel, p_direction, p_length);
    }

    void setupFifo(const dma::DmaBurstSize_t /* p_burst */, const dma::DmaFifoThreshold_t /* p_threshold */) const {
    }

    void setupSource(const void * const /* p_addr */, const unsigned /* p_width */, const bool /* p_incr */) const {
        // this->m_stream.setupSource(p_addr, p_width, p_incr);
    }

    void setupTarget(void * const /* p_addr */, const unsigned /* p_width */, const bool /* p_incr */) const {
        // this->m_stream.setupTarget(p_addr, p_width, p_incr);    
    }

    void start(/* const DmaChannelCallback * const p_callback */ void) const {
        // assert(this->m_callback == NULL);
        // this->m_callback = p_callback;
        // this->m_stream.enable(&this->m_streamCallback);
    }

    void stop(void) const {
        // this->m_stream.disable();    
        // this->m_callback = NULL;
    }
};
/*****************************************************************************/

/*****************************************************************************/
    } /* namespace f1 */
} /* namespace dma */
/*****************************************************************************/

#endif /* _DMA_CHANNEL_STM32_HPP_B6BD9823_B66D_4724_9A96_965E03319555 */
