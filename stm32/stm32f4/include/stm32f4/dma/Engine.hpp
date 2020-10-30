/*-
 * $Copyright$
-*/

#ifndef _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67
#define _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67

#include <stdint.h>
#include <stddef.h>

#include <stm32/dma/Types.hpp>

#include "stm32f4xx.h"

/*****************************************************************************/
namespace stm32 {
    namespace dma {
/*****************************************************************************/

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

/*****************************************************************************/
class DmaEngine {
public:
    void setupSource(DMA_Stream_TypeDef * const p_stream, const void * const p_addr, const unsigned p_width, const bool p_incr) const {
        const DmaDirection_t direction = static_cast<DmaDirection_t>((p_stream->CR & DMA_SxCR_DIR) >> 6);

        assert((p_width == 1) || (p_width == 2) || (p_width == 4));
        const unsigned width = p_width / 2;

        unsigned mask = 0, clr = 0;

        switch (direction) {
        case e_PeripheralToMemory:
            p_stream->PAR = reinterpret_cast<intptr_t>(p_addr);
            clr  |= DMA_SxCR_PSIZE | DMA_SxCR_PINC;
            mask |= (width << 11) & DMA_SxCR_PSIZE;
            mask |= p_incr ? DMA_SxCR_PINC : 0;
            break;
        case e_MemoryToPeripheral:
            /* FALLTHRU */
        case e_MemoryToMemory:
            p_stream->M0AR = reinterpret_cast<intptr_t>(p_addr);
            clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
            mask |= (width << 13) & DMA_SxCR_MSIZE;
            mask |= p_incr ? DMA_SxCR_MINC : 0;
            break;
        default:
            assert(direction != e_Undefined);
        }

        p_stream->CR &= ~clr;
        p_stream->CR |= mask;
    }

    void setupTarget(DMA_Stream_TypeDef * const p_stream, void * const p_addr, const unsigned p_width, const bool p_incr) const {
        const DmaDirection_t direction = static_cast<DmaDirection_t>((p_stream->CR & DMA_SxCR_DIR) >> 6);

        assert((p_width == 1) || (p_width == 2) || (p_width == 4));
        const unsigned width = p_width / 2;

        unsigned mask = 0, clr = 0;

        switch (direction) {
        case e_PeripheralToMemory:
            p_stream->M0AR = reinterpret_cast<intptr_t>(p_addr);
            clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
            mask |= (width << 13) & DMA_SxCR_MSIZE;
            mask |= p_incr ? DMA_SxCR_MINC : 0;
            break;
        case e_MemoryToPeripheral:
            p_stream->PAR = reinterpret_cast<intptr_t>(p_addr);
            clr  |= DMA_SxCR_PSIZE | DMA_SxCR_PINC;
            mask |= p_incr ? DMA_SxCR_PINC : 0;
            break;
        case e_MemoryToMemory:
            p_stream->M1AR = reinterpret_cast<intptr_t>(p_addr);
            clr  |= DMA_SxCR_MSIZE | DMA_SxCR_MINC;
            mask |= (width << 13) & DMA_SxCR_MSIZE;
            mask |= p_incr ? DMA_SxCR_MINC : 0;
            break;
        default:
            assert(direction != e_Undefined);
        }

        p_stream->CR &= ~clr;
        p_stream->CR |= mask;
    }

    void setupFifo(DMA_Stream_TypeDef * const p_stream, const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
        if (p_burst != e_Single) {
            p_stream->FCR &= ~DMA_SxFCR_FTH;
            p_stream->FCR |= DMA_SxFCR_DMDIS | p_threshold | DMA_SxFCR_FEIE;

            p_stream->CR &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);
            p_stream->CR |= ((p_burst << 23) | (p_burst << 21));
        } else {
            p_stream->FCR &= ~DMA_SxFCR_DMDIS;

            p_stream->CR &= ~(DMA_SxCR_MBURST | DMA_SxCR_PBURST);
            p_stream->CR |= DMA_SxCR_DMEIE;
        }
    }

    void enable(DMA_Stream_TypeDef * p_stream, const unsigned p_streamNo) const {
        this->handleStreamIrq(p_stream, p_streamNo);
        p_stream->CR |= DMA_SxCR_EN;
    }

    void disable(DMA_Stream_TypeDef * p_stream, const unsigned p_streamNo) const {
        uint8_t flags = this->handleStreamIrq(p_stream, p_streamNo);
        assert(!(flags & ~e_Complete));
        (void) flags;

        p_stream->CR &= ~DMA_SxCR_EN;
    }

    uint8_t handleStreamIrq(DMA_Stream_TypeDef * const p_stream, const unsigned p_streamNo) const {
        unsigned bitoffs;
        uint8_t flags;

        switch (p_streamNo) {
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
            assert(p_streamNo < 8);
            bitoffs = 28; /* Silence "maybe used uninitialized" warning */
            break;
        }

        if (p_streamNo > 3) { /* Streams 4 - 7 */
            flags = (m_dma.HISR >> bitoffs) & 0x3F;
            m_dma.HIFCR |= flags << bitoffs;
        } else { /* Streams 0 - 3 */
            flags = (m_dma.LISR >> bitoffs) & 0x3F;
            m_dma.LIFCR |= flags << bitoffs;
        }

        /*
        * Per Datasheet section 9.3.18, a "FIFO Error" may not be an actual error.
        * The way  I read the datasheet is to ignore the FIFO errors in modes that
        * involve peripherals and let the peripheral detect the error.
        */
        if ((flags & e_FifoError) && ((p_stream->CR & DMA_SxCR_DIR) != DMA_SxCR_DIR_1)) {
            flags &= ~e_FifoError;
        }
        
        return flags;
    }

protected:
    DmaEngine(DMA_TypeDef & p_dma, unsigned p_engine)
      : m_dma(p_dma), m_engine(p_engine) {
          (void) m_engine;
    }

    ~DmaEngine() {

    }

private:
    DMA_TypeDef &  m_dma;
    const unsigned m_engine;
}; /* class DmaEngine */
/*****************************************************************************/

/*****************************************************************************/
template<
    typename RccT,
    intptr_t Address
>
class DmaEngineT : public EngineT<Address>, public DmaEngine {
    const RccT &m_rcc;

    enum class Dma_t {
        e_Dma1 = 0,
        e_Dma2 = 1,
        e_Invalid
    };

    static constexpr
    Dma_t getDmaEngine(intptr_t p_address) {
        return (
            p_address == DMA1_BASE ? Dma_t::e_Dma1
          : p_address == DMA2_BASE ? Dma_t::e_Dma2
          : Dma_t::e_Invalid
        );
    }

public:
    DmaEngineT(RccT &p_rcc)
      : DmaEngine(* reinterpret_cast<DMA_TypeDef *>(Address), static_cast<unsigned>(getDmaEngine(Address))), m_rcc(p_rcc) {
        static_assert(getDmaEngine(Address) != Dma_t::e_Invalid);
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));
    }

     ~DmaEngineT() {
        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
     }

    static constexpr
    intptr_t
    getBaseAddr(void) {
        return Address;
    }

    static constexpr
    intptr_t
    getStreamBaseAddr(unsigned p_stream) {
        return (Address + 0x10 + p_stream * 0x18);
    }
};
/*****************************************************************************/

/*****************************************************************************/
    } /* namespace stm32 */
} /* namespace dma */
/*****************************************************************************/

#endif /* _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67 */
