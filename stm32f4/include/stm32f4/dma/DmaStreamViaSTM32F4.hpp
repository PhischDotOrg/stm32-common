/*-
 * $Copyright$
-*/

#ifndef _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0
#define _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0

#include <stdint.h>
#include <stddef.h>

#include <dma/DmaEngineViaSTM32F4.hpp>
#include <dma/DmaTypesViaSTM32F4.hpp>

namespace dma {

template<typename DmaEngineT = typename dma::DmaEngineViaSTM32F4>
class DmaStreamViaSTM32F4T {
public:
    class Callback {
    public:
        virtual ~Callback() {};

        virtual void notify(const DmaTransferStatus_t p_status) const = 0;
    };

    DmaStreamViaSTM32F4T(DmaEngineT &p_dma, const unsigned p_number);
    ~DmaStreamViaSTM32F4T();

   void setup(const unsigned p_channel, const DmaDirection_t p_direction, const size_t p_length) const;

   void setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const;
   void setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const;
   
   void setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const;
   
   void enable(const Callback * const p_callback);
   void disable(void);

   void setPriority(const unsigned p_priority) const;

   void handleIrq(void) const;

   typename DmaEngineT::Dma_t getDmaEngine(void) const { return m_dma.getDmaEngine(); }
   unsigned getDmaStream(void) const { return m_stream; }

private:
    DmaEngineT &        m_dma;
    const unsigned      m_stream;
    const Callback *    m_callback;
};

typedef class DmaStreamViaSTM32F4T<> DmaStreamViaSTM32F4;

} /* namespace dma */

// #include "dma/DmaStreamViaSTM32F4.cpp"

#endif /* _DMA_STREAM_STM32F4_ee105d91_78de_4b77_960f_0295358183c0 */
