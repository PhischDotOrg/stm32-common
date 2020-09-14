/*-
 * $Copyright$
-*/
#ifndef _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f
#define _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f

#include <stdint.h>
#include <stddef.h>

#include <stm32/dma/Types.hpp>
#include <stm32/dma/Stream.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace dma {
/*****************************************************************************/

/*****************************************************************************/
class DmaChannelCallback {
public:
    virtual ~DmaChannelCallback() {};

    virtual void notify(const DmaTransferStatus_t p_status) const = 0;
};
/*****************************************************************************/

/*****************************************************************************/
template<typename DmaStreamT>
class ChannelT {
public:
    using Callback = DmaChannelCallback;

    ChannelT(DmaStreamT &p_stream, const unsigned p_channel)
      : m_stream(p_stream), m_channel(p_channel), m_streamCallback(*this), m_callback(NULL) {

    }

    ~ChannelT() {

    }

    void setup(const DmaDirection_t p_direction, const size_t p_length) const {
        this->m_stream.disable();
        this->m_stream.setup(this->m_channel, p_direction, p_length);
    }

    void setupSource(const void * const p_addr, const unsigned p_width, const bool p_incr) const {
        this->m_stream.setupSource(p_addr, p_width, p_incr);
    }

    void setupTarget(void * const p_addr, const unsigned p_width, const bool p_incr) const {
        this->m_stream.setupTarget(p_addr, p_width, p_incr);    
    }

    void setupFifo(const DmaBurstSize_t p_burst, const DmaFifoThreshold_t p_threshold) const {
        this->m_stream.setupFifo(p_burst, p_threshold);
    }
   
    void start(const DmaChannelCallback * const p_callback) {
        assert(this->m_callback == NULL);
        this->m_callback = p_callback;
        this->m_stream.enable(&this->m_streamCallback);
    }

    void stop(void) {
        this->m_stream.disable();    
        this->m_callback = NULL;
    }

    void setPriority(const unsigned p_priority) const {
        this->m_stream.setPriority(p_priority);
    }

private:
    void disable(void) const;
    void enable(void) const;

    void notify(const DmaTransferStatus_t p_status) const {
        assert(this->m_callback != NULL);
        this->m_callback->notify(p_status);
    }

    class StreamCallback : public DmaStreamCallback {
    private:
        const ChannelT &m_obj;

    public:
        StreamCallback(const ChannelT &p_obj) : m_obj(p_obj) {
        }

        virtual ~StreamCallback() {
        }

        virtual void notify(const DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status);
        }
    };

    DmaStreamT &                m_stream;
    const unsigned              m_channel;
    StreamCallback              m_streamCallback;
    const DmaChannelCallback *  m_callback;
};
/*****************************************************************************/

/*****************************************************************************/
    } /* namespace stm32 */
} /* namespace dma */
/*****************************************************************************/

#endif /* _DMA_CHANNEL_STM32F4_HPP_89514427_dd70_425d_814c_b5b317e73c5f */