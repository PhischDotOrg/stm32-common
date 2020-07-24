/*-
 * $Copyright$
-*/

#ifndef _ADC_STM32F4_CPP_d9460d00_ba44_4468_b39c_d5c570bee91a
#define _ADC_STM32F4_CPP_d9460d00_ba44_4468_b39c_d5c570bee91a

#include <stm32f4/AdcViaSTM32F4.hpp>
#include <assert.h>

#define ADC_RETURNCODE(_x) (0x10000000 | ((_x) << 16) | __LINE__)

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
AdcViaSTM32F4Base::AdcViaSTM32F4Base(ADC_TypeDef * const p_adc)
  : m_adc(p_adc), m_adcError(0), m_externalTrigger(e_AdcTriggerEnable_None), m_channels(0) {
    this->m_event = xSemaphoreCreateCounting(8, 0);
}

/*******************************************************************************
 * 
 ******************************************************************************/
AdcViaSTM32F4Base::~AdcViaSTM32F4Base() {
    vSemaphoreDelete(this->m_event);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupChannel(const AdcViaSTM32F4Channel_t p_channel,
  const AdcViaSTM32F4Rank_t p_rank, const AdcViaSTM32F4SampleTime_t p_sampleTime) {
    this->m_channels |= 1 << p_channel;

    unsigned smpr_offs;
    volatile uint32_t *smpr;

    unsigned sqr_offs;
    volatile uint32_t *sqr;

    if (p_channel < e_AdcChannel_10) {
        smpr_offs = 3 * p_channel;
        smpr = &this->m_adc->SMPR2;
    } else {
        smpr_offs = 3 * (p_channel - e_AdcChannel_10);
        smpr = &this->m_adc->SMPR1;
    }

    *smpr &= ~(7 << smpr_offs);
    *smpr |= p_sampleTime << smpr_offs;
    
    if (p_rank < 7) {
        sqr_offs = 4 * p_rank;
        sqr = &this->m_adc->SQR3;
    } else if (p_rank < 13) {
        sqr_offs = 4 * (p_rank - e_AdcRank_7);
        sqr = &this->m_adc->SQR2;
    } else {
        sqr_offs = 4 * (p_rank - e_AdcRank_13);
        sqr = &this->m_adc->SQR1;
    }
    
    *sqr &= ~(0x1F << sqr_offs);
    *sqr |= p_channel << sqr_offs;
    
    this->setupChannelSequenceLength();
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupChannelSequenceLength(void) const {
    unsigned cnt = 0;

    for (unsigned i = 0; i < sizeof(this->m_channels) * 8; i++) {
        if (this->m_channels & (1 << i))
            cnt++;
    }
    
    assert(cnt < 17);

    this->m_adc->SQR1 &= ~ADC_SQR1_L;
    this->m_adc->SQR1 |= (cnt - 1) << 20;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
AdcViaSTM32F4Base::clearChannelSequence(void) {
    this->m_channels = 0;

    this->m_adc->SQR1 = 0;
    this->m_adc->SQR2 = 0;
    this->m_adc->SQR3 = 0;
    this->m_adc->SMPR1 = 0;
    this->m_adc->SMPR2 = 0;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupExternalTrigger(const AdcViaSTM32F4Trigger_t p_trigger, const AdcViaSTM32F4TriggerEnable_t p_enable) {
    /*
     * Don't write the trigger enable to the device, yet, as that would trigger
     * conversions right away. Instead, let the sample() method do the actual
     * enablement.
     */
    this->m_externalTrigger = p_enable;
    
    this->m_adc->CR2 &= ~ADC_CR2_EXTSEL;
    this->m_adc->CR2 |= p_trigger << 24;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::handleIrq(void) {
    this->m_adcError = this->m_adc->SR;

    if (this->m_adcError & ADC_SR_OVR) {
        this->disableOverrunIrq();
    }

    if (this->m_adcError & ADC_SR_EOC) {
        this->disableEocIrq();
    };
    
    this->wakeUp();
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::enableExternalTrigger(void) const {
    this->setupExternalTrigger(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::disableExternalTrigger(void) const {
    this->setupExternalTrigger(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupExternalTrigger(const bool p_enable) const {
    this->m_adc->CR2 &= ~ADC_CR2_EXTEN;
    if (p_enable) {
        this->m_adc->CR2 |= this->m_externalTrigger << 28;
    } else {
        this->m_adc->CR2 &= ~(this->m_externalTrigger << 28);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::enableOverrunIrq(void) const {
    this->setupOverrunIrq(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::disableOverrunIrq(void) const {
    this->m_adc->SR |= ADC_SR_OVR;
    this->setupOverrunIrq(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupOverrunIrq(const bool p_enable) const {
    this->m_adc->CR1 &= ~ADC_CR1_OVRIE;
    if (p_enable) {
        this->m_adc->CR1 |= ADC_CR1_OVRIE;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::enableEocIrq(void) const {
    this->setupEocIrq(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::disableEocIrq(void) const {
    this->m_adc->SR |= ADC_SR_EOC;
    this->setupEocIrq(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupEocIrq(const bool p_enable) const {
    this->m_adc->CR1 &= ~ADC_CR1_EOCIE;
    if (p_enable) {
        this->m_adc->CR1 |= ADC_CR1_EOCIE;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::startConversion(void) const {
    this->m_adc->CR2 |= ADC_CR2_SWSTART;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupResolution(const AdcViaSTM32F4Resolution_t p_resolution) const {
    this->m_adc->CR1 &= ~ADC_CR1_RES;
    this->m_adc->CR1 |= p_resolution << 24;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::enable(void) const {
    this->m_adc->CR2 |= ADC_CR2_ADON;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::disable(void) const {
    this->m_adc->CR2 &= ~ADC_CR2_ADON;
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::wakeUp(void) const {
    BaseType_t higherPrioTaskWoken;
    if (xSemaphoreGiveFromISR(this->m_event, &higherPrioTaskWoken) == pdTRUE) {
        portYIELD_FROM_ISR(higherPrioTaskWoken);
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::enableDma(void) const {
    this->setupDma(true);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::disableDma(void) const {
    this->setupDma(false);
}

/*******************************************************************************
 *
 ******************************************************************************/
void
AdcViaSTM32F4Base::setupDma(const bool p_enable) const {
    const uint32_t mask = ADC_CR2_DMA | ADC_CR2_DDS;

    this->m_adc->CR2 &= ~mask;
    if (p_enable) {
        this->m_adc->CR2 |= ADC_CR2_DMA;
    }
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaChannelT>
AdcViaSTM32F4T<DmaChannelT>::AdcViaSTM32F4T(ADC_TypeDef * const p_adc, DmaChannelT &p_dmaChannel)
  : AdcViaSTM32F4Base(p_adc), m_dmaChannel(p_dmaChannel), m_dmaChannelCallback(*this), m_dmaStatus(dma::e_DmaUndefined) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaChannelT>
AdcViaSTM32F4T<DmaChannelT>::~AdcViaSTM32F4T(void) {

}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaChannelT>
int
AdcViaSTM32F4T<DmaChannelT>::sample(void * const p_buffer, const size_t p_size, const size_t p_count, const unsigned p_sampleTimeoutInMs /* = 0 */) {
    assert(p_buffer != NULL);
    assert(p_count > 0);
    assert(p_size > 0 && p_size <= sizeof(uint32_t));
    int rc = 0;

    this->m_dmaStatus = dma::e_DmaUndefined;
    this->m_adcError = 0;

    this->enableExternalTrigger();

    this->enableOverrunIrq();
    this->enableDma();

    this->m_dmaChannel.setup(dma::e_PeripheralToMemory, p_count);
    this->m_dmaChannel.setupSource(const_cast<const uint32_t * const>(&(this->m_adc->DR)), p_size, false);
    this->m_dmaChannel.setupTarget(p_buffer, p_size, true);
    this->m_dmaChannel.start(&this->m_dmaChannelCallback);

    this->enable();

    do {
        if (!xSemaphoreTake(this->m_event, p_sampleTimeoutInMs ? p_sampleTimeoutInMs / portTICK_PERIOD_MS : portMAX_DELAY)) {
            rc = ADC_RETURNCODE(1);
            assert(false);
            goto out;
        }

        if (this->m_adcError != 0) {
            rc = ADC_RETURNCODE(2);
            break;
        }

        if (this->m_dmaStatus == dma::e_DmaError) {
            rc = ADC_RETURNCODE(3);
            break;
        }
    } while(this->m_dmaStatus != dma::e_DmaComplete);

out:
    this->disable();

    this->m_dmaChannel.stop();

    this->disableDma();
    this->enableOverrunIrq();

    this->disableExternalTrigger();

    return (rc);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<typename DmaChannelT>
int
AdcViaSTM32F4T<DmaChannelT>::sample(uint16_t * const p_value, const unsigned p_sampleTimeoutInMs /* = 0 */) {
    assert(p_value != NULL);

    int rc = 0;

    this->disableDma();
    this->disableExternalTrigger();

    this->m_adcError = 0;
    *p_value = this->m_adc->DR;

    this->disableOverrunIrq();

    this->enableEocIrq();
    this->enable();
    
    this->startConversion();

    if (!xSemaphoreTake(this->m_event, p_sampleTimeoutInMs ? p_sampleTimeoutInMs / portTICK_PERIOD_MS : portMAX_DELAY)) {
        rc = ADC_RETURNCODE(1);
        assert(false);
        goto out;
    }

    if (this->m_adcError & ADC_SR_EOC) {
        *p_value = this->m_adc->DR;
    } else {
        rc = this->m_adcError;
    }

out:
    this->disable();
    this->disableEocIrq();

    return(rc);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
void
AdcViaSTM32F4T<DmaChannelT>::notify(const dma::DmaTransferStatus_t p_status) {
    this->m_dmaStatus = p_status;
    this->wakeUp();
}

} /* namespace devices */

#endif /* _ADC_STM32F4_CPP_d9460d00_ba44_4468_b39c_d5c570bee91a */
