/*-
 * $Copyright$
-*/

#ifndef _NVIC_STM32F4_CPP_b11a355a_03cd_40fd_aa3b_493d0752aaa3
#define _NVIC_STM32F4_CPP_b11a355a_03cd_40fd_aa3b_493d0752aaa3

#include <stm32f4/NvicViaSTM32F4.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename ScbT>
NvicViaSTM32F4T<ScbT>::NvicViaSTM32F4T(NVIC_Type *p_nvic, ScbT &p_scb)
  : NvicViaSTM32F4Base(p_nvic), m_scb(p_scb) {
    m_scb.configurePriorityGroup(ScbT::e_PriorityGroup_4);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename ScbT>
NvicViaSTM32F4T<ScbT>::~NvicViaSTM32F4T(void) {

}

/*******************************************************************************
 * 
 ******************************************************************************/
NvicViaSTM32F4Base::NvicViaSTM32F4Base(NVIC_Type * p_nvic) : m_nvic(p_nvic) {
}

/*******************************************************************************
 * 
 ******************************************************************************/
NvicViaSTM32F4Base::~NvicViaSTM32F4Base(void) {

}

#if defined(HOSTBUILD)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 0
#endif

/*******************************************************************************
 * 
 ******************************************************************************/
void
NvicViaSTM32F4Base::enableIrqWithPriorityGroup(const Irq_t p_irq, const unsigned p_priorityGroup) const {
    m_nvic->IP[p_irq] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (p_priorityGroup - 3)) << (8 - __NVIC_PRIO_BITS);
    m_nvic->ISER[p_irq >> 0x05] = 1 << (p_irq & 0x1F);
}

/*******************************************************************************
 * 
 ******************************************************************************/
void
NvicViaSTM32F4Base::disableIrq(const Irq_t p_irq) const {
    m_nvic->ISER[p_irq >> 0x05] &= ~(1 << (p_irq & 0x1F));
}

/*******************************************************************************
 * 
 ******************************************************************************/
const NvicViaSTM32F4Base::Irq_t NvicViaSTM32F4Base::m_dmaIrq[2][8] = {
    { /* DMA-1 Streams */
        DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn,
        DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn
    },
    { /* DMA-2 Streams */
        DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn,
        DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn
    }
};

template<typename ScbT>
void
NvicViaSTM32F4T<ScbT>::enableIrq(const dma::DmaStreamViaSTM32F4 &p_obj) const {
    this->enableIrqWithPriorityGroup(m_dmaIrq[p_obj.getDmaEngine()][p_obj.getDmaStream()]);
}

template class NvicViaSTM32F4T<devices::ScbViaSTM32F4>;

} /* namespace devices */

#endif /* _NVIC_STM32F4_CPP_b11a355a_03cd_40fd_aa3b_493d0752aaa3 */
