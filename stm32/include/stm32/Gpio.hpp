/*-
 * $Copyright$
 */

#ifndef _STM32_GPIO_HPP_846A1A66_29B4_4842_AEE6_722DB81CA052
#define _STM32_GPIO_HPP_846A1A66_29B4_4842_AEE6_722DB81CA052

#include <stm32/Engine.hpp>
#include <stm32f4xx.h>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
template<
    typename PinConfigurationPolicyT
>
class GpioEngineT : public PinConfigurationPolicyT {
    GPIO_TypeDef &m_engine;

public:
    using Mode_e = typename PinConfigurationPolicyT::Mode_e;
    using Termination_e = typename PinConfigurationPolicyT::Termination_e;
    using Function_e = typename PinConfigurationPolicyT::Function_e;

    static constexpr unsigned m_width = 16;

    void
    disable(const uint8_t p_pin) const {
        this->enable(p_pin, Mode_e::e_Input, Termination_e::e_None);
    }

    void
    enable(uint8_t p_pin, Mode_e p_mode, Termination_e p_termination) const {
        PinConfigurationPolicyT::enable(m_engine, p_pin, p_mode, p_termination);
    }

    void
    write(uint16_t p_value, uint16_t /* p_output */, uint16_t p_mask) const {
        m_engine.BSRR = (((~p_value & p_mask) << 16) & 0xFFFF0000)
                      | ((( p_value & p_mask) <<  0) & 0x0000FFFF);
    }

    void
    read(uint16_t &p_vector) const {
        p_vector = m_engine.IDR;
    }

    template<typename EngineT>
    void
    selectAlternateFn(uint8_t p_pin, const EngineT &p_engine) const {
        PinConfigurationPolicyT::selectAlternateFn(this->m_engine, p_pin, p_engine);
    }

protected:
    GpioEngineT(GPIO_TypeDef * const p_engine) : m_engine(*p_engine) {

    }
}; /* class GpioEngineT */

/*****************************************************************************/

/*****************************************************************************/
template<
    typename RccT,
    intptr_t Address,
    typename PinConfigurationPolicyT
>
class GpioT : public EngineT<Address>, public GpioEngineT<PinConfigurationPolicyT> {
    const RccT &m_rcc;

public:
    constexpr GpioT(const RccT &p_rcc)
      : GpioEngineT<PinConfigurationPolicyT>(reinterpret_cast<GPIO_TypeDef *>(Address)), m_rcc(p_rcc) {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));
    }

    ~GpioT() {
        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    }
}; /* class GpioT */

/*
 * Assert that class type really is just the size of two pointers, i.e. that
 * the base class EngineT<> does not add to the size of the object
 */
// static_assert(sizeof(GpioT<void *, 0>) == 2 * sizeof(void *));
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_GPIO_HPP_846A1A66_29B4_4842_AEE6_722DB81CA052 */
