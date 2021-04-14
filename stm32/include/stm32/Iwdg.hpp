/*-
 * $Copyright$
 */
#ifndef _STM32_IWDG_HPP_7645B750_5D52_4C2B_9264_EFEDF15DD854
#define _STM32_IWDG_HPP_7645B750_5D52_4C2B_9264_EFEDF15DD854

#include <stm32/Engine.hpp>
#include <stm32f4xx.h>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
class IwdgEngine {
private:
    IWDG_TypeDef &  m_iwdg;

    enum class Key_e : uint16_t {
        e_Lock      = 0x0000,
        e_Unlock    = 0x5555,
        e_Reload    = 0xAAAA,
        e_Start     = 0xCCCC,
    };

    enum class Prescaler_e : uint8_t {
        e_Div4      = 0b000,
        e_Div8      = 0b001,
        e_Div16     = 0b010,
        e_Div32     = 0b011,
        e_Div64     = 0b100,
        e_Div128    = 0b101,
        e_Div256    = 0b110,
    };

protected:
    void
    setPrescaler(const Prescaler_e p_divider) const {
        while ((m_iwdg.SR & IWDG_SR_PVU) != 0) __NOP();

        unlock();
        m_iwdg.PR = static_cast<uint16_t>(p_divider);
        lock();
    }

    void
    lock(void) const {
        m_iwdg.KR = static_cast<uint16_t>(Key_e::e_Lock);
    }

    void
    unlock(void) const {
        m_iwdg.KR = static_cast<uint16_t>(Key_e::e_Unlock);
    }

    void
    writeKr(const Key_e p_value) const {
        unlock();
        m_iwdg.KR = static_cast<uint16_t>(p_value);
        lock();
    }

    void
    setReloadReg(const unsigned p_value) {
        assert(p_value < (1 << 12));

        unlock();
        m_iwdg.RLR = p_value & 0xFFF;
        lock();
    }

protected:
    IwdgEngine(IWDG_TypeDef * const p_iwdg)
      : m_iwdg(*p_iwdg) {

    }

    ~IwdgEngine() = default;

public:
    void reload(void) const {
        writeKr(Key_e::e_Reload);
    }

    void
    start(const unsigned /* p_timeoutInMs */ = 1000) {
        setPrescaler(Prescaler_e::e_Div32);
        setReloadReg(0xFFF);

        writeKr(Key_e::e_Start);
    }
};
/*****************************************************************************/

/*****************************************************************************/
template<
    typename RccT,
    intptr_t Address
>
class IwdgT : public EngineT<Address>, public IwdgEngine {
    const RccT &m_rcc;

public:
    IwdgT(const RccT &p_rcc) : IwdgEngine(reinterpret_cast<IWDG_TypeDef *>(Address)), m_rcc(p_rcc) {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));
    }

    ~IwdgT() = default;
}; /* class IwdgT */
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_IWDG_HPP_7645B750_5D52_4C2B_9264_EFEDF15DD854 */
