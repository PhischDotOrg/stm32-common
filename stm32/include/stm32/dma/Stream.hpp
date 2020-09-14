/*-
 * $Copyright$
-*/

#ifndef _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0
#define _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0

#include <stdint.h>
#include <stddef.h>

#include <stm32/Engine.hpp>
#include <stm32/dma/Types.hpp>
#include <stm32/dma/Engine.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace dma {
/*****************************************************************************/

/*****************************************************************************/
class DmaStreamCallback {
public:
    virtual ~DmaStreamCallback() {};

    virtual void notify(const DmaTransferStatus_t p_status) const = 0;
};
/*****************************************************************************/

/*****************************************************************************/
template<
    typename NvicT,
    typename DmaEngineT,
    unsigned nStream
>
class StreamT : public EngineT< DmaEngineT::getStreamBaseAddr(nStream) >
{
    const NvicT &               m_nvic;
    DmaEngineT &                m_dma;
    DMA_Stream_TypeDef * const  m_stream;
    const unsigned              m_streamNo;

public:
    StreamT(const NvicT &p_nvic, DmaEngineT &p_dma)
      : m_nvic(p_nvic),
        m_dma(p_dma),
        m_stream(reinterpret_cast<DMA_Stream_TypeDef *>(this->m_engineType)),
        m_streamNo(nStream),
        m_callback(NULL)
    {
        p_nvic.enableIrq(* static_cast< EngineT< DmaEngineT::getStreamBaseAddr(nStream) > *>(this));
    }

    ~StreamT() {
        m_nvic.disableIrq(* static_cast< EngineT< DmaEngineT::getStreamBaseAddr(nStream) > *>(this));
    }

    void setup(const unsigned p_channel, const DmaDirection_t p_direction, const size_t p_length) const {
        this->m_dma.disable(this->m_stream, nStream);

        uint32_t mask = 0;

        mask |= (p_channel << 25);
        mask |= (p_direction << 6);
        mask |= (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE);

        if (!(m_stream->FCR & DMA_SxFCR_DMDIS)) {
            mask |= DMA_SxCR_DMEIE;
        }

        m_stream->CR &= ~mask;
        m_stream->CR |= mask;

        m_stream->NDTR = p_length;
    }

    void setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const {
        this->m_dma.setupSource(this->m_stream, p_addr, p_width, p_incr);
    }

    void setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const {
        this->m_dma.setupTarget(this->m_stream, p_addr, p_width, p_incr);
    }

    void setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
        this->m_dma.setupFifo(this->m_stream, p_burst, p_threshold);
    }

    void enable(const DmaStreamCallback * const p_callback) {
        assert(this->m_callback == NULL);
        this->m_callback = p_callback;
        this->m_dma.enable(this->m_stream, nStream);
    }

    void disable(void) {
        this->m_dma.disable(this->m_stream, nStream);
        this->m_callback = NULL;
    }

    void setPriority(const unsigned p_priority) const {
        assert(p_priority < 4);

        m_stream->CR &= DMA_SxCR_PL;
        m_stream->CR |= p_priority << 16;
    }

    void handleIrq(void) const {
        const uint8_t status = this->m_dma.handleStreamIrq(this->m_stream, this->m_streamNo);

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

private:
    const DmaStreamCallback *   m_callback;
};
/*****************************************************************************/

/*****************************************************************************/
    } /* namespace stm32 */
} /* namespace dma */
/*****************************************************************************/

#endif /* _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0 */
