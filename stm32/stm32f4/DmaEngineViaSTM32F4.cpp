/*-
 * $Copyright$
-*/

#include <dma/DmaEngineViaSTM32F4.hpp>

#include <assert.h>
#include <stdint.h>

#include "stm32f4xx.h"

struct DMA_Stream_TypeDef;

namespace dma {

/*******************************************************************************
 *
 ******************************************************************************/
static DMA_Stream_TypeDef * const g_dmaStreams[2][8] = {
#if defined(DMA1_Stream0)
    { /* DMA-1 Streams */
        DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3,
        DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7
    },
    { /* DMA-2 Streams */
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3,
        DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7            
    }
#endif
};

#if !defined(HOSTBUILD)
static const IRQn_Type g_dmaIrq[2][8] = {
#if defined(DMA1_Stream0_IRQn)
    { /* DMA-1 Streams */
        DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
        DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn
    },
    { /* DMA-2 Streams */
        DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
        DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
    }
#endif
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
DmaEngineViaSTM32F4::DmaEngineViaSTM32F4(const Dma_t p_engine, DMA_TypeDef * const p_dma)
  : m_engine(p_engine), m_dma(p_dma) {
}

/*******************************************************************************
 *
 ******************************************************************************/
DmaEngineViaSTM32F4::~DmaEngineViaSTM32F4() {
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::setup(const unsigned p_stream, const unsigned p_channel, const DmaDirection_t p_direction, const size_t p_length) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];
    assert(p_channel < 8);

    uint32_t mask = 0;

    mask |= (p_channel << 25);
    mask |= (p_direction << 6);
    mask |= (DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE);
    
    if (!(stream->FCR & DMA_SxFCR_DMDIS)) {
        mask |= DMA_SxCR_DMEIE;
    }

    stream->CR &= ~mask;
    stream->CR |= mask;
    
    stream->NDTR = p_length;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::setupSource(const unsigned p_stream, const void * const p_addr, const unsigned p_width, const bool p_incr) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];
    const DmaDirection_t direction = static_cast<DmaDirection_t>((stream->CR & DMA_SxCR_DIR) >> 6);

    assert((p_width == 1) || (p_width == 2) || (p_width == 4));
    const unsigned width = p_width / 2;

    unsigned mask = 0, clr = 0;

    switch (direction) {
    case e_PeripheralToMemory:
        stream->PAR = reinterpret_cast<intptr_t>(p_addr);
        clr  |= DMA_SxCR_PSIZE | DMA_SxCR_PINC;
        mask |= (width << 11) & DMA_SxCR_PSIZE;
        mask |= p_incr ? DMA_SxCR_PINC : 0;
        break;
    case e_MemoryToPeripheral:
        /* FALLTHRU */
    case e_MemoryToMemory:
        stream->M0AR = reinterpret_cast<intptr_t>(p_addr);
        clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
        mask |= (width << 13) & DMA_SxCR_MSIZE;
        mask |= p_incr ? DMA_SxCR_MINC : 0;
        break;
    default:
        assert(direction != e_Undefined);
    }

    stream->CR &= ~clr;
    stream->CR |= mask;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::setupTarget(const unsigned p_stream, void * const p_addr, const unsigned p_width, const bool p_incr) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];
    const DmaDirection_t direction = static_cast<DmaDirection_t>((stream->CR & DMA_SxCR_DIR) >> 6);

    assert((p_width == 1) || (p_width == 2) || (p_width == 4));
    const unsigned width = p_width / 2;

    unsigned mask = 0, clr = 0;

    switch (direction) {
    case e_PeripheralToMemory:
        stream->M0AR = reinterpret_cast<intptr_t>(p_addr);
        clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
        mask |= (width << 13) & DMA_SxCR_MSIZE;
        mask |= p_incr ? DMA_SxCR_MINC : 0;
        break;
    case e_MemoryToPeripheral:
        stream->PAR = reinterpret_cast<intptr_t>(p_addr);
        clr  |= DMA_SxCR_PSIZE | DMA_SxCR_PINC;
        mask |= p_incr ? DMA_SxCR_PINC : 0;
        break;
    case e_MemoryToMemory:
        stream->M1AR = reinterpret_cast<intptr_t>(p_addr);
        clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
        mask |= (width << 13) & DMA_SxCR_MSIZE;
        mask |= p_incr ? DMA_SxCR_MINC : 0;
        break;
    default:
        assert(direction != e_Undefined);
    }    

    stream->CR &= ~clr;
    stream->CR |= mask;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::setupFifo(const unsigned p_stream, const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];

    if (p_burst != e_Single) {
        stream->FCR &= ~DMA_SxFCR_FTH;
        stream->FCR |= DMA_SxFCR_DMDIS | p_threshold | DMA_SxFCR_FEIE;
        
        stream->CR &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);
        stream->CR |= ((p_burst << 23) | (p_burst << 21));        
    } else {
        stream->FCR &= ~DMA_SxFCR_DMDIS;
        
        stream->CR &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);
        stream->CR |= DMA_SxCR_DMEIE;        
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::enable(const unsigned p_stream) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];

    this->handleStreamIrq(p_stream);
    stream->CR |= DMA_SxCR_EN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::disable(const unsigned p_stream) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];

    uint8_t flags = this->handleStreamIrq(p_stream);
    assert(!(flags & ~e_Complete));

    stream->CR &= ~DMA_SxCR_EN;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
DmaEngineViaSTM32F4::setPriority(const unsigned p_stream, const unsigned p_priority) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];

    assert(p_priority < 4);

    stream->CR &= DMA_SxCR_PL;
    stream->CR |= p_priority << 16;
}

/*******************************************************************************
 *
 ******************************************************************************/
uint8_t
DmaEngineViaSTM32F4::handleStreamIrq(const unsigned p_stream) const {
    DMA_Stream_TypeDef * const stream = g_dmaStreams[this->m_engine][p_stream];

    unsigned bitoffs;
    uint8_t flags;

    switch (p_stream) {
    case 0:
    case 4:
        bitoffs = 0;
        break;
    case 1:
    case 5:
        bitoffs = 6;
        break;
    case 2:
    case 6:
        bitoffs = 16;
        break;
    case 3:
    case 7:
        bitoffs = 22;
        break;
    default:
        assert(p_stream < 8);
        bitoffs = 28; /* Silence "maybe used uninitialized" warning */
        break;
    }

    if (p_stream > 3) { /* Streams 4 - 7 */
        flags = (this->m_dma->HISR >> bitoffs) & 0x3F;
        this->m_dma->HIFCR |= flags << bitoffs;
    } else { /* Streams 0 - 3 */
        flags = (this->m_dma->LISR >> bitoffs) & 0x3F;
        this->m_dma->LIFCR |= flags << bitoffs;
    }

    /*
     * Per Datasheet section 9.3.18, a "FIFO Error" may not be an actual error.
     * The way  I read the datasheet is to ignore the FIFO errors in modes that
     * involve peripherals and let the peripheral detect the error.
     */
    if ((flags & e_FifoError) && ((stream->CR & DMA_SxCR_DIR) != DMA_SxCR_DIR_1)) {
        flags &= ~e_FifoError;
    }
    
    return flags;
}

} /* namespace dma */
