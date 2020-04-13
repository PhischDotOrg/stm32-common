/*-
 * $Copyright$
-*/

#ifndef _DMA_CHANNEL_STM32F4_CPP_13d0402e_6ecc_4399_b0bf_aaa2ea2c96f3
#define _DMA_CHANNEL_STM32F4_CPP_13d0402e_6ecc_4399_b0bf_aaa2ea2c96f3

#include <dma/DmaChannelViaSTM32F4.hpp>
#include <assert.h>

namespace dma {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
DmaChannelViaSTM32F4T<DmaStreamT>::DmaChannelViaSTM32F4T(DmaStreamT &p_stream, const unsigned p_channel)
  : m_stream(p_stream), m_channel(p_channel), m_streamCallback(*this), m_callback(NULL) {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
DmaChannelViaSTM32F4T<DmaStreamT>::~DmaChannelViaSTM32F4T() {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::setup(const DmaDirection_t p_direction, const size_t p_length) const {
    this->m_stream.disable();
    this->m_stream.setup(this->m_channel, p_direction, p_length);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const {
    this->m_stream.setupSource(p_addr, p_width, p_incr);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const {
    this->m_stream.setupTarget(p_addr, p_width, p_incr);    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
    this->m_stream.setupFifo(p_burst, p_threshold);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::start(const Callback * const p_callback) {
    assert(this->m_callback == NULL);
    this->m_callback = p_callback;
    this->m_stream.enable(&this->m_streamCallback);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::stop(void) {
    this->m_stream.disable();    
    this->m_callback = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::setPriority(const unsigned p_priority) const {
    this->m_stream.setPriority(p_priority);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaStreamT>
void
DmaChannelViaSTM32F4T<DmaStreamT>::notify(const DmaTransferStatus_t p_status) const {
    assert(this->m_callback != NULL);
    this->m_callback->notify(p_status);
}

} /* namespace dma */

#endif /* _DMA_CHANNEL_STM32F4_CPP_13d0402e_6ecc_4399_b0bf_aaa2ea2c96f3 */
