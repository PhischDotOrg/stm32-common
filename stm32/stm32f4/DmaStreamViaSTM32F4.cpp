/*-
 * $Copyright$
-*/

#ifndef _DMA_STREAM_STM32F4_HPP_549a14b1_2aa9_46c5_8eb2_2b0a879861ec
#define _DMA_STREAM_STM32F4_HPP_549a14b1_2aa9_46c5_8eb2_2b0a879861ec

#include <dma/DmaStreamViaSTM32F4.hpp>
#include <assert.h>

namespace dma {

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
DmaStreamViaSTM32F4T<DmaEngineT>::DmaStreamViaSTM32F4T(DmaEngineT &p_dma, const unsigned p_number)
  : m_dma(p_dma), m_stream(p_number), m_callback(NULL) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
DmaStreamViaSTM32F4T<DmaEngineT>::~DmaStreamViaSTM32F4T() {
    
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::setup(const unsigned p_channel, const DmaDirection_t p_direction, const size_t p_length) const {
    this->m_dma.disable(this->m_stream);
    this->m_dma.setup(this->m_stream, p_channel, p_direction, p_length);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const {
    this->m_dma.setupSource(this->m_stream, p_addr, p_width, p_incr);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const {
    this->m_dma.setupTarget(this->m_stream, p_addr, p_width, p_incr);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
    this->m_dma.setupFifo(this->m_stream, p_burst, p_threshold);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::enable(const Callback * const p_callback) {
    assert(this->m_callback == NULL);
    this->m_callback = p_callback;
    this->m_dma.enable(this->m_stream);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::disable(void) {
    this->m_dma.disable(this->m_stream);
    this->m_callback = NULL;
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::setPriority(const unsigned p_priority) const {
    this->m_dma.setPriority(this->m_stream, p_priority);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaEngineT>
void
DmaStreamViaSTM32F4T<DmaEngineT>::handleIrq(void) const {
    const uint8_t status = this->m_dma.handleStreamIrq(this->m_stream);

    assert(this->m_callback != NULL);

    if(status & (e_FifoError | e_DirectModeError | e_TransferError)) {
        /* Error during transfer */
        this->m_callback->notify(e_DmaError);
    } else if (status & e_Complete) {
        /* Successful transfer */
        this->m_callback->notify(e_DmaComplete);
    } else if (status & e_HalfTransfer) {
        this->m_callback->notify(e_DmaInProgress);  
    } else {
        /* Spurious interrupt, or maybe a FIFO error that has been filtered out */
    }
}

} /* namespace dma */

#endif /* _DMA_STREAM_STM32F4_HPP_549a14b1_2aa9_46c5_8eb2_2b0a879861ec */
