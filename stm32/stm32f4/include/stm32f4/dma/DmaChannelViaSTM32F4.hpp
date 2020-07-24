/*-
 * $Copyright$
-*/
#ifndef _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f
#define _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f

#include <stdint.h>
#include <stddef.h>

#include <dma/DmaStreamViaSTM32F4.hpp>
#include <dma/DmaTypesViaSTM32F4.hpp>

namespace dma {

template<typename DmaStreamT = dma::DmaStreamViaSTM32F4>
class DmaChannelViaSTM32F4T {
public:
    class Callback {
    public:
        virtual ~Callback() {};

        virtual void notify(const DmaTransferStatus_t p_status) const = 0;
    };
    
    DmaChannelViaSTM32F4T(DmaStreamT &p_stream, const unsigned p_channel);
    ~DmaChannelViaSTM32F4T();

   void setup(const DmaDirection_t p_direction, const size_t p_length) const;

   void setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const;
   void setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const;

   void setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const;
   
   void start(const Callback * const p_callback);
   void stop(void);

   void setPriority(const unsigned p_priority) const;

private:
    void disable(void) const;
    void enable(void) const;
    void notify(const DmaTransferStatus_t p_status) const;

    class StreamCallback : public DmaStreamT::Callback {
    private:
        const DmaChannelViaSTM32F4T &m_obj;

    public:
        StreamCallback(const DmaChannelViaSTM32F4T &p_obj) : m_obj(p_obj) {
        }

        virtual ~StreamCallback() {
        }

        virtual void notify(const DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status);
        }
    };

    DmaStreamT &        m_stream;
    const unsigned      m_channel;
    StreamCallback      m_streamCallback;
    const Callback *    m_callback;
};

typedef class DmaChannelViaSTM32F4T<> DmaChannelViaSTM32F4;

} /* namespace dma */

#include "DmaChannelViaSTM32F4.cpp"

#endif /* _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f */