/*-
 * $Copyright$
 */

#ifndef _RCC_ENGINE_HPP_E3F52548_5B31_4C0D_AAFF_4A55F892080D
#define _RCC_ENGINE_HPP_E3F52548_5B31_4C0D_AAFF_4A55F892080D

#include <stm32/Engine.hpp>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

template<typename EngineT> struct BusTypeT {
    static constexpr decltype(nullptr) m_busType = nullptr;
};

#define MAP_RCC_ENGINE(Engine)                                  \
template<> struct BusTypeT< EngineT< (Engine ## _BASE) > > {    \
    static constexpr auto m_busType = Rcc::e_##Engine;          \
}

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _RCC_ENGINE_HPP_E3F52548_5B31_4C0D_AAFF_4A55F892080D */