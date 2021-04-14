/*-
 * $Copyright$
 */
#ifndef _STM32_DBGMCU_HPP_C528738F_9A71_46BA_B7DE_1EC296EB4D1E
#define _STM32_DBGMCU_HPP_C528738F_9A71_46BA_B7DE_1EC296EB4D1E

#include <stm32/Engine.hpp>

#include <stdint.h>

#include "stm32f4xx.h"

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
class DbgMcu {
    // DBGMCU_TypeDef &m_dbgMcu;
public:
    DbgMcu(DBGMCU_TypeDef &p_dbgMcu) /* : m_dbgMcu(p_dbgMcu) */ {
        p_dbgMcu.CR |= DBGMCU_CR_DBG_IWDG_STOP; // Stop the Independent Watchdog when the Core is halted for debugging.
    }
}; /* class DbgMcu */
/*****************************************************************************/

/*****************************************************************************/
template<
    uintptr_t AddressT,
    typename PinT
>
class DbgMcuT : public DbgMcu, public EngineT<AddressT> {
    const PinT &    m_swoPin;
public:
    DbgMcuT(const PinT &p_swoPin) : DbgMcu(* reinterpret_cast<DBGMCU_TypeDef *>(AddressT)), m_swoPin(p_swoPin) {

    }

    void
    enableSwo(void) const {
        m_swoPin.selectAlternateFn(* static_cast<const EngineT<AddressT> *>(this));
    }
};
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/
#endif /* _STM32_DBGMCU_HPP_C528738F_9A71_46BA_B7DE_1EC296EB4D1E */