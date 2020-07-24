/*-
 * $Copyright$
 */
#ifndef _STM32F1_NVIC_HPP_6B303303_3356_4A58_BF43_0D4361A826E6
#define _STM32F1_NVIC_HPP_6B303303_3356_4A58_BF43_0D4361A826E6

namespace stm32 {
    namespace f1 {
        template<typename ScbT>
        class NvicT {
            NVIC_Type &     m_nvic;
            const ScbT &    m_scb;

        public:
            constexpr NvicT(NVIC_Type * const p_nvic, const ScbT &p_scb) : m_nvic(*p_nvic), m_scb(p_scb) {
            
            }
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_NVIC_HPP_6B303303_3356_4A58_BF43_0D4361A826E6 */
