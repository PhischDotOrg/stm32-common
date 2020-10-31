/*-
 * $Copyright$
 */
#ifndef _STM32_COREDEBUG_HPP_6D835092_8D80_4175_8B13_674757B88B01
#define _STM32_COREDEBUG_HPP_6D835092_8D80_4175_8B13_674757B88B01

#include <stm32/Engine.hpp>

#include <stdint.h>
#include "core_cm4.h"

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
class CoreDbg {
    /*
     * Documentation on the ARM Core Debug Registers can be found at:
     *
     *  Cortex M3: https://developer.arm.com/documentation/ddi0337/e/core-debug/about-core-debug?lang=en
     *  Cortex M4: https://developer.arm.com/documentation/ddi0439/b/Debug/About-debug/Debug-register-summary?lang=en
     *    --> References the ARMv7-M Architecture Reference Manual that can be downloaded here: https://developer.arm.com/documentation/ddi0403/ed/?lang=en
     *    Section "C1.6 Debug system registers" contains the relevant documentation.
     */
    CoreDebug_Type    &m_coreDebug;

    void
    enable(void) const {
        /* Enable DWT and ITM units. */
        m_coreDebug.DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    void
    disable(void) const {
        /* Disable DWT and ITM units. */
        m_coreDebug.DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    }

public:
    CoreDbg(CoreDebug_Type &p_coreDebug) : m_coreDebug(p_coreDebug) {

    }

    void
    enableDwt(void) const {
        enable();
    }

    void
    enableItm(void) const {
        enable();
    }

    void
    enableTpi(void) const {
        enable();
    }
}; /* class CoreDbg */
/*****************************************************************************/

/*****************************************************************************/
// Note: "CoreDebug" is a Macro in core_cm4.hpp
template<
    uintptr_t AddressT
>
class CoreDbgT : public EngineT<AddressT>, public CoreDbg {

public:
    constexpr CoreDbgT(void) : CoreDbg(* reinterpret_cast<CoreDebug_Type *>(this->m_engineType)) {

    }
}; /* class CoreDebugT */
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_COREDEBUG_HPP_6D835092_8D80_4175_8B13_674757B88B01 */
