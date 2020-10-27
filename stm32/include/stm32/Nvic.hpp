/*-
 * $Copyright$
 */
#ifndef _NVIC_HPP_3AE10C0C_DEEB_4B5B_8B48_F8D6E2465116
#define _NVIC_HPP_3AE10C0C_DEEB_4B5B_8B48_F8D6E2465116

#include <stm32f4xx.h>

#include <stm32/RccEngine.hpp>

#include <FreeRTOSConfig.h>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
template<typename EngineT> struct IrqTypeT {
    static constexpr std::nullptr_t m_irq = nullptr;
};

#define MAP_NVIC_ENGINE_IRQ(Engine, Irq)                                    \
template<> struct IrqTypeT< ::stm32::EngineT< (Engine ## _BASE) > > {       \
    static constexpr auto m_irq = NvicBase::Irq_t:: Irq ## _IRQn;           \
}

#define MAP_NVIC_IRQ(Engine)                                                \
template<> struct IrqTypeT< ::stm32::EngineT< (Engine ## _BASE) > > {       \
    static constexpr auto m_irq = NvicBase::Irq_t:: Engine ## _IRQn;        \
}
/*****************************************************************************/

/*****************************************************************************/
class NvicBase {
    NVIC_Type &     m_nvic;

public:
    using Irq_t = ::IRQn_Type;

    void disableIrq(const Irq_t p_irq) const {
        m_nvic.ISER[p_irq >> 0x05] &= ~(1 << (p_irq & 0x1F));
    }

protected:
    constexpr NvicBase(NVIC_Type &p_nvic) : m_nvic(p_nvic) {

    }

    void enableIrqWithPriorityGroup(const Irq_t p_irq, const unsigned p_priorityGroup) const {
        m_nvic.IP[p_irq] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (p_priorityGroup - 3)) << (8 - __NVIC_PRIO_BITS);
        m_nvic.ISER[p_irq >> 0x05] = 1 << (p_irq & 0x1F);
    }

public:
};
/*****************************************************************************/

/*****************************************************************************/
template<typename ScbT>
class NvicT : public ::stm32::NvicBase {
    const ScbT &    m_scb;

public:
    constexpr NvicT(NVIC_Type * const p_nvic, const ScbT &p_scb) : NvicBase(*p_nvic), m_scb(p_scb) {
        m_scb.configurePriorityGroup(ScbT::e_PriorityGroup_4);
    }

    void enableIrqWithPriorityGroup(const Irq_t p_irq) const {
        NvicBase::enableIrqWithPriorityGroup(p_irq, static_cast<const unsigned>(this->m_scb.getPriorityGroup()));
    }

    template<typename EngineT>
    void
    enableIrq(const EngineT & /* p_engine */) const {
        this->enableIrqWithPriorityGroup(IrqTypeT<EngineT>::m_irq);
    }

    template<typename EngineT>
    void
    disableIrq(const EngineT & /* p_engine */) const {
        this->disableIrq(IrqTypeT<EngineT>::m_irq);
    }
};
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _NVIC_HPP_3AE10C0C_DEEB_4B5B_8B48_F8D6E2465116 */
