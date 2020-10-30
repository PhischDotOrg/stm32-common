/*-
 * $Copyright$
 */
#ifndef _STM32_ENGINE_HPP_C3783B32_41BE_4ACD_8195_7968014AB16E
#define _STM32_ENGINE_HPP_C3783B32_41BE_4ACD_8195_7968014AB16E

#include <cstdint>

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

template<uintptr_t EngineAddress>
struct EngineT {
    enum : uintptr_t {
        m_engineType = EngineAddress
    };

protected:
    ~EngineT() = default;
};

/*
 * The above type declaration should only be used to dispatch types at compile
 * time. Test that the type declaration doesn't allocate any space.
 */
static_assert(sizeof(EngineT<0>) == 1);

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_ENGINE_HPP_C3783B32_41BE_4ACD_8195_7968014AB16E */
