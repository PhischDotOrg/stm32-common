/*-
 * $Copyright$
-*/

#ifndef _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67
#define _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67

#include <stdint.h>
#include <stddef.h>

#include <dma/DmaTypesViaSTM32F4.hpp>
#include <stm32f4/RccViaSTM32F4.hpp>

namespace dma {

/*******************************************************************************
 *
 ******************************************************************************/
class DmaEngineViaSTM32F4 {
public:
    typedef enum Dma_e {
        e_Dma1 = 0,
        e_Dma2 = 1
    } Dma_t;

   Dma_t getDmaEngine(void) const { return this->m_engine; };

   void setup(const unsigned p_stream, const unsigned p_channel, const DmaDirection_t p_direction, const size_t p_length) const;

   void setupSource(const unsigned p_stream, const void * const p_addr, const unsigned p_width, const bool p_incr) const;
   void setupTarget(const unsigned p_stream, void * const p_addr, const unsigned p_width, const bool p_incr) const;

   void setupFifo(const unsigned p_stream, const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const;
   
   void enable(const unsigned p_stream) const;
   void disable(const unsigned p_stream) const;

   void setPriority(const unsigned p_stream, const unsigned p_priority) const;
   
   uint8_t handleStreamIrq(const unsigned p_stream) const;

protected:
    DmaEngineViaSTM32F4(const Dma_t p_engine, DMA_TypeDef * const p_dma);
    ~DmaEngineViaSTM32F4();

private:
    const Dma_t         m_engine;
    DMA_TypeDef * const m_dma;
};

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t> struct DmaEngineViaSTM32F4DmaFunction;

template<> struct DmaEngineViaSTM32F4DmaFunction<DMA1_BASE> {
    static const DmaEngineViaSTM32F4::Dma_t m_dmaType = DmaEngineViaSTM32F4::e_Dma1;
    static const devices::RccViaSTM32F4::FunctionAHB1_t m_rccType = devices::RccViaSTM32F4::e_Dma1;
};

template<> struct DmaEngineViaSTM32F4DmaFunction<DMA2_BASE> {
    static const DmaEngineViaSTM32F4::Dma_t m_dmaType = DmaEngineViaSTM32F4::e_Dma2;
    static const devices::RccViaSTM32F4::FunctionAHB1_t m_rccType = devices::RccViaSTM32F4::e_Dma2;
};

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t DmaEngineT, typename RccT = devices::RccViaSTM32F4>
class DmaEngineViaSTM32F4T : public DmaEngineViaSTM32F4 {
public:
    DmaEngineViaSTM32F4T(RccT &p_rcc)
      : DmaEngineViaSTM32F4(DmaEngineViaSTM32F4DmaFunction<DmaEngineT>::m_dmaType, reinterpret_cast<DMA_TypeDef *>(DmaEngineT)), m_rcc(p_rcc) {
        this->m_rcc.enable(DmaEngineViaSTM32F4DmaFunction<DmaEngineT>::m_rccType);
    }

     ~DmaEngineViaSTM32F4T() {
        this->m_rcc.disable(DmaEngineViaSTM32F4DmaFunction<DmaEngineT>::m_rccType);
     }

protected:
    RccT &      m_rcc;
};

typedef DmaEngineViaSTM32F4T<DMA1_BASE, devices::RccViaSTM32F4> DmaEngineViaSTM32F4_Dma1;
typedef DmaEngineViaSTM32F4T<DMA2_BASE, devices::RccViaSTM32F4> DmaEngineViaSTM32F4_Dma2;

} /* namespace dma */

#endif /* _DMA_ENGINE_STM32F4_HPP_7fddc595_a3e3_4049_b535_97d908fc9b67 */
