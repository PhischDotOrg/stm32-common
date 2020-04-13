/*-
 * $Copyright$
-*/

#ifndef _ADC_STM32F4_HPP_248e23c1_47e3_4402_b5f2_f2422fee70e5
#define _ADC_STM32F4_HPP_248e23c1_47e3_4402_b5f2_f2422fee70e5

#include <dma/DmaChannel.hpp>
#include <stm32f4/RccViaSTM32F4.hpp>
#include <timer/TimerChannelViaSTM32F4.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */
#include <FreeRTOS.h>
#include <semphr.h>

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>
#include <stddef.h>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class AdcViaSTM32F4Types {
public:
    typedef enum AdcViaSTM32F4Channel_e {
        e_AdcChannel_0  = 0,
        e_AdcChannel_1  = 1,
        e_AdcChannel_2  = 2,
        e_AdcChannel_3  = 3,
        e_AdcChannel_4  = 4,
        e_AdcChannel_5  = 5,
        e_AdcChannel_6  = 6,
        e_AdcChannel_7  = 7,
        e_AdcChannel_8  = 8,
        e_AdcChannel_9  = 9,
        e_AdcChannel_10 = 10,
        e_AdcChannel_11 = 11,
        e_AdcChannel_12 = 12,
        e_AdcChannel_13 = 13,
        e_AdcChannel_14 = 14,
        e_AdcChannel_15 = 15,
        e_AdcChannel_16 = 16,
        e_AdcChannel_17 = 17,
        e_AdcChannel_18 = 18,
    } AdcViaSTM32F4Channel_t;
    
    typedef enum AdcViaSTM32F4Rank_e {
        e_AdcRank_1     = 0x00,
        e_AdcRank_2     = 0x01,
        e_AdcRank_3     = 0x02,
        e_AdcRank_4     = 0x03,
        e_AdcRank_5     = 0x04,
        e_AdcRank_6     = 0x05,
        e_AdcRank_7     = 0x06,
        e_AdcRank_8     = 0x07,
        e_AdcRank_9     = 0x08,
        e_AdcRank_10    = 0x09,
        e_AdcRank_11    = 0x0A,
        e_AdcRank_12    = 0x0B,
        e_AdcRank_13    = 0x0C,
        e_AdcRank_14    = 0x0D,
        e_AdcRank_15    = 0x0E,
        e_AdcRank_16    = 0x0F,
        e_AdcRank_17    = 0x10,
    } AdcViaSTM32F4Rank_t;
    
    typedef enum AdcViaSTM32F4SampleTime_e {
        e_ADC_SampleTime_3Cycles    = 0x0,
        e_ADC_SampleTime_15Cycles   = 0x1,
        e_ADC_SampleTime_28Cycles   = 0x2,
        e_ADC_SampleTime_56Cycles   = 0x3,
        e_ADC_SampleTime_84Cycles   = 0x4,
        e_ADC_SampleTime_112Cycles  = 0x5,
        e_ADC_SampleTime_144Cycles  = 0x6,
        e_ADC_SampleTime_480Cycles  = 0x7,
    } AdcViaSTM32F4SampleTime_t;
    
    typedef enum AdcViaSTM32F4Trigger_e {
        e_AdcTrigger_Timer1_CC1     = 0x0,
        e_AdcTrigger_Timer1_CC2     = 0x1,
        e_AdcTrigger_Timer1_CC3     = 0x2,
        e_AdcTrigger_Timer2_CC2     = 0x3,
        e_AdcTrigger_Timer2_CC3     = 0x4,
        e_AdcTrigger_Timer2_CC4     = 0x5,
        e_AdcTrigger_Timer2_TRGO    = 0x6,
        e_AdcTrigger_Timer3_CC1     = 0x7,
        e_AdcTrigger_Timer3_TRGO    = 0x8,
        e_AdcTrigger_Timer4_CC4     = 0x9,
        e_AdcTrigger_Timer5_CC1     = 0xA,
        e_AdcTrigger_Timer5_CC2     = 0xB,
        e_AdcTrigger_Timer5_CC3     = 0xC,
        e_AdcTrigger_Timer8_CC1     = 0xD,
        e_AdcTrigger_Timer8_TRGO    = 0xE,
        e_AdcTrigger_Ext_IT11       = 0xF,
    } AdcViaSTM32F4Trigger_t;

    typedef enum AdcViaSTM32F4TriggerEnable_e {
        e_AdcTriggerEnable_None             = 0x0,
        e_AdcTriggerEnable_RisingEdge       = 0x1,
        e_AdcTriggerEnable_FallingEdge      = 0x2,
        e_AdcTriggerEnable_BothEdges        = 0x3,
    } AdcViaSTM32F4TriggerEnable_t;

    typedef enum AdcViaSTM32F4Resolution_e {
        e_AdcResolution_12Bit   = 0x0,
        e_AdcResolution_10Bit   = 0x1,
        e_AdcResolution_8Bit    = 0x2,
        e_AdcResolution_6Bit    = 0x3,
    } AdcViaSTM32F4Resolution_t;
    
protected:
    AdcViaSTM32F4Types(void) {};
    ~AdcViaSTM32F4Types() {};
};

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t TimerT, timer::TimerViaSTM32F4::TimerChannel_t> struct AdcViaSTM32F4TriggerT;

template<> struct AdcViaSTM32F4TriggerT<TIM4_BASE, timer::TimerViaSTM32F4::e_TimerViaSTM32F4_Channel4> {
    static const AdcViaSTM32F4Types::AdcViaSTM32F4Trigger_t m_trigger = AdcViaSTM32F4Types::e_AdcTrigger_Timer4_CC4;
};

/*******************************************************************************
 * 
 ******************************************************************************/
class AdcViaSTM32F4Base : public AdcViaSTM32F4Types {
public:
    void handleIrq(void);

    void setupChannel(const AdcViaSTM32F4Channel_t p_channel, const AdcViaSTM32F4Rank_t p_rank, const AdcViaSTM32F4SampleTime_t p_sampleTime);
    void clearChannelSequence();

    void setupExternalTrigger(const AdcViaSTM32F4Trigger_t p_trigger, const AdcViaSTM32F4TriggerEnable_t p_enable);

    template<intptr_t TimerT, timer::TimerViaSTM32F4::TimerChannel_t ChannelT>    
    void setupExternalTrigger(const timer::TimerOutputChannelViaSTM32F4T<TimerT, ChannelT> & /*p_timerChannel*/, const AdcViaSTM32F4TriggerEnable_t p_enable) {
        AdcViaSTM32F4Trigger_t trigger = AdcViaSTM32F4TriggerT<TimerT, ChannelT>::m_trigger;

        this->setupExternalTrigger(trigger, p_enable);
    };

    void setupResolution(const AdcViaSTM32F4Resolution_t p_resolution) const;

protected:
    AdcViaSTM32F4Base(ADC_TypeDef * const p_adc);
    ~AdcViaSTM32F4Base();

    ADC_TypeDef * const             m_adc;
    uint16_t                        m_adcError;

    SemaphoreHandle_t               m_event;

    void enableDma(void) const;
    void disableDma(void) const;
    
    void wakeUp(void) const;
    
    void enableOverrunIrq(void) const;
    void disableOverrunIrq(void) const;

    void enableExternalTrigger(void) const;
    void disableExternalTrigger(void) const;

    void enableEocIrq(void) const;
    void disableEocIrq(void) const;

    void startConversion(void) const;

    void enable(void) const;
    void disable(void) const;

private:
    AdcViaSTM32F4TriggerEnable_t    m_externalTrigger;

    uint32_t                        m_channels;

    void setupExternalTrigger(const bool p_enable) const;
    void setupOverrunIrq(const bool p_enable) const;
    void setupEocIrq(const bool p_enable) const;
    void setupDma(const bool p_enable) const;
    void setupChannelSequenceLength(void) const;
};

/*******************************************************************************
 * 
 ******************************************************************************/
typedef typename dma::DmaChannelViaSTM32F4T<> DefaultDmaChannel_t;

template<typename DmaChannelT = DefaultDmaChannel_t >
class AdcViaSTM32F4T : public AdcViaSTM32F4Base {
private:
    class DmaChannelCallback : public DmaChannelT::Callback {
    private:
        AdcViaSTM32F4T &m_obj;

    public:
        DmaChannelCallback(AdcViaSTM32F4T &p_obj) : m_obj(p_obj) {};
        virtual ~DmaChannelCallback() {};

        virtual void notify(const dma::DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status);
        }
    };

    void notify(const dma::DmaTransferStatus_t p_status);
    
    DmaChannelT &                   m_dmaChannel;
    DmaChannelCallback              m_dmaChannelCallback;
    dma::DmaTransferStatus_t        m_dmaStatus;

protected:
    AdcViaSTM32F4T(ADC_TypeDef * const p_adc, DmaChannelT &p_dmaChannel);
    ~AdcViaSTM32F4T();
    
public:
    int sample(void * const p_buffer, const size_t p_size, const size_t p_count, const unsigned p_sampleTimeoutInMs = 0);
    int sample(uint16_t * const p_value, const unsigned p_sampleTimeoutInMs = 0);
};

typedef AdcViaSTM32F4T<> AdcViaSTM32F4;

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT = DefaultDmaChannel_t, typename RccT = RccViaSTM32F4>
class AdcViaSTM32F4_Adc1T : public AdcViaSTM32F4T<DmaChannelT> {
private:
    RccT &m_rcc;

public:
    AdcViaSTM32F4_Adc1T(RccT &p_rcc, DmaChannelT &p_dmaChannel) : AdcViaSTM32F4T<DmaChannelT>(ADC1, p_dmaChannel), m_rcc(p_rcc) {
        m_rcc.enable(RccT::e_Adc1);
    }
    ~AdcViaSTM32F4_Adc1T() {
        m_rcc.disable(RccT::e_Adc1);
    };
};

typedef AdcViaSTM32F4_Adc1T<> AdcViaSTM32F4_Adc1;

} /* namespace devices */

#endif /* _ADC_STM32F4_HPP_248e23c1_47e3_4402_b5f2_f2422fee70e5 */
