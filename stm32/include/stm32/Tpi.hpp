/*-
 * $Copyright$
 */
#ifndef _STM32_TPIU_HPP_74F6D8A0_A22F_46BD_85EF_713165A23254
#define _STM32_TPIU_HPP_74F6D8A0_A22F_46BD_85EF_713165A23254

#include <stm32/Engine.hpp>

#include "core_cm4.h"

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
// Note: "TPI" is a Macro in core_cm4.hpp
class Tpi {
    /*
     * Documentation on ITM_Type's Registers can be found at:
     *
     *  Cortex M3: TODO 
     *  Cortex M4: https://developer.arm.com/documentation/ddi0439/b/Trace-Port-Interface-Unit/TPIU-programmers-model?lang=en
     *
     * Some of the Bits are implementation defined and are documented in the CPU Reference Manual by STMicroelectronics.
     */
    TPI_Type    &m_tpi;

public:
   enum class TracePortProtocol_e : uint8_t {
        e_Sync              = 0b00,
        e_SWO_Manchester    = 0b01,
        e_SWO_NRZ           = 0b10
    };

    void
    setDivisor(uint32_t p_divisor) const {
        m_tpi.ACPR = p_divisor;
    }

    void
    setTracePortProtocol(TracePortProtocol_e p_tracePortProtocol) const {
        m_tpi.SPPR &= ~(TPI_SPPR_TXMODE_Msk);
        m_tpi.SPPR |= (static_cast<uint32_t>(p_tracePortProtocol) << TPI_SPPR_TXMODE_Pos);
    }

    void
    enableFormatter(void) const {
        m_tpi.FFCR |= TPI_FFCR_EnFCont_Pos;
    }

    void
    disableFormatter(void) const {
        m_tpi.FFCR &= ~TPI_FFCR_EnFCont_Msk;
    }

    void
    setParallelTracePortWidth(uint32_t p_bits) const {
        assert(p_bits > 0);
        assert(p_bits <= 32);

        uint32_t setMask = (1 << (p_bits - 1));

        uint32_t supportedWidths = m_tpi.SSPSR;
        assert((supportedWidths & setMask) == setMask);

        if ((supportedWidths & setMask) == setMask) {
            m_tpi.CSPSR  = setMask;
        }
    }

    Tpi(TPI_Type &p_tpi) : m_tpi(p_tpi) {

    }
}; /* class Tpi */
/*****************************************************************************/

/*****************************************************************************/
template<
    uintptr_t AddressT,
    typename CoreDbgT,
    typename PinT
>
class TpiT : public EngineT<AddressT>, public Tpi {

public:
    TpiT(const CoreDbgT &p_coreDbg, PinT &p_swo)
      : Tpi(* reinterpret_cast<TPI_Type *>(this->m_engineType)) {
        p_coreDbg.enableTpi();

        p_swo.selectAlternateFn(* static_cast<EngineT<AddressT> *>(this));
    }
}; /* class Itm */
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_ITM_HPP_17543F25_1525_4FAC_8CDA_6691E88C7157 */
