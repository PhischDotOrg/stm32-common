/*-
 * $Copyright$
-*/

#ifndef _DMA_ENGINE_STM32_HPP_626A6236_150F_474F_B5B6_718D421EB363
#define _DMA_ENGINE_STM32_HPP_626A6236_150F_474F_B5B6_718D421EB363

#include <stdint.h>
#include <stddef.h>

#include <stm32/Engine.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
/*****************************************************************************/

template<
    typename RccT,
    intptr_t Address
>
class DmaEngineT : public EngineT<Address> {
    const RccT &    m_rcc;

public:
    DmaEngineT(const RccT &p_rcc) : m_rcc(p_rcc) {
        m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));
    }

    ~DmaEngineT() {
        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    }
};

/*****************************************************************************/
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _DMA_ENGINE_STM32_HPP_626A6236_150F_474F_B5B6_718D421EB363 */
