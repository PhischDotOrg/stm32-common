/*-
 * $Copyright$
 */
#ifndef _STM32_ITM_HPP_17543F25_1525_4FAC_8CDA_6691E88C7157
#define _STM32_ITM_HPP_17543F25_1525_4FAC_8CDA_6691E88C7157

#include <stm32/Engine.hpp>

#include "stm32f4xx.h"

/*****************************************************************************/
namespace stm32 {
/*****************************************************************************/

/*****************************************************************************/
// Note: "ITM" is a Macro in core_cm4.hpp
class Itm {
    static constexpr uint32_t m_key = 0xC5ACCE55u;

    /*
     * Documentation on ITM_Type's Registers can be found at:
     *
     *  Cortex M3: https://developer.arm.com/documentation/ddi0337/e/system-debug/itm/summary-and-description-of-the-itm-registers
     *  Cortex M4: https://developer.arm.com/documentation/ddi0439/b/Instrumentation-Trace-Macrocell-Unit/ITM-programmers-model?lang=en
     * 
     * Some of the Bits are implementation defined and are documented in the CPU Reference Manual by STMicroelectronics.
     * See Section 23.14 ITM (instrumentation trace macrocell).
     * 
     */
    ITM_Type &      m_itm;

    void
    unlock(void) const {
        m_itm.LAR   = m_key;
    }

    void
    setAtbId(uint8_t p_atbId) const {
        assert(p_atbId > 0);
        assert(p_atbId <= 0x7F);

        m_itm.TCR &= ~ITM_TCR_TraceBusID_Msk;
        m_itm.TCR |= (p_atbId << ITM_TCR_TraceBusID_Pos) & ITM_TCR_TraceBusID_Msk;
    }

    void
    enableSWO(void) const {
        m_itm.TCR |= ITM_TCR_SWOENA_Msk;
    }

    void
    disableSWO(void) const {
        m_itm.TCR &= ~ITM_TCR_SWOENA_Msk;
    }

    void
    enableDwtSync(void) const {
        m_itm.TCR |= ITM_TCR_SYNCENA_Msk;
    }

    void
    disableDwtSync(void) const {
        m_itm.TCR &= ~ITM_TCR_SYNCENA_Msk;
    }

    /*
     * The "enable" operation should be done by the Debugger (if connected). Otherwise,
     * the sendPort() method will block infinitely waiting for the FIFO to clear when
     * no debugger is connected.
     */
    // void
    // enable(void) const {
    //     m_itm.TCR |= ITM_TCR_ITMENA_Msk;
    // }

    bool
    isEnabled(void) const {
        return (m_itm.TCR & ITM_TCR_ITMENA_Msk);
    }

    void
    disable(void) const {
        m_itm.TCR &= ~ITM_TCR_ITMENA_Msk;
    }

    enum class PrivilegePortMask_e : uint8_t {
        e_Ports_31_24   = 0b1000,
        e_Ports_23_16   = 0b0100,
        e_Ports_15_8    = 0b0010,
        e_Ports_7_0     = 0b0001,
    };

    void
    enablePrivilege(PrivilegePortMask_e p_ports) const {
        m_itm.TPR |= static_cast<uint8_t>(p_ports);
    }

    void
    disablePrivilege(PrivilegePortMask_e p_ports) const {
        m_itm.TPR &= ~(static_cast<uint8_t>(p_ports));
    }

    bool
    isStimulusPortEnabled(uint8_t p_stimulusPort) const {
        return m_itm.TER & (1 << p_stimulusPort);
    }

public:
    Itm(ITM_Type &p_itm) : m_itm(p_itm) {
        unlock();

        setAtbId(1);

        enableSWO();
        enableDwtSync();
    }

    static constexpr
    uint32_t
    getDivisor(uint32_t p_clkSpeed, uint32_t p_baudRate = 2'250'000) {
        return ((p_clkSpeed / p_baudRate) - 1);
    }

    void
    sendPort(uint8_t p_port, char p_char) const {
        if (this->isEnabled() && this->isStimulusPortEnabled(p_port)) {
            while (m_itm.PORT[p_port].u32 == 0);

            m_itm.PORT[p_port].u8 = p_char;
        }
    }

    void
    enableStimulusPort(uint8_t p_stimulusPort) const {
        if (p_stimulusPort < 8) {
            enablePrivilege(PrivilegePortMask_e::e_Ports_7_0);
        } else if (p_stimulusPort < 16) {
            enablePrivilege(PrivilegePortMask_e::e_Ports_15_8);
        } else if (p_stimulusPort < 24) {
            enablePrivilege(PrivilegePortMask_e::e_Ports_23_16);
        } else if (p_stimulusPort < 32) {
            enablePrivilege(PrivilegePortMask_e::e_Ports_31_24);
        } else {
            assert(p_stimulusPort <= 31);
        }

        m_itm.TER |= (1 << p_stimulusPort);
    }
}; /* class Itm */
/*****************************************************************************/

/*****************************************************************************/
template<
    uintptr_t AddressT,
    typename TpiT
>
class ItmT : public EngineT<AddressT>, public Itm {
public:
    ItmT(const TpiT &p_tpi, uint32_t p_divisor)
      : Itm(* reinterpret_cast<ITM_Type *>(this->m_engineType)) {
        p_tpi.enableFormatter();
        p_tpi.setTracePortProtocol(TpiT::TracePortProtocol_e::e_SWO_NRZ);
        p_tpi.setParallelTracePortWidth(1);
        p_tpi.setDivisor(p_divisor);
    }
};
/*****************************************************************************/

/*****************************************************************************/
class ItmPort {
    const Itm &     m_itm;
    const unsigned  m_port;

public:
    constexpr ItmPort(const Itm &p_itm, unsigned p_port)
      : m_itm(p_itm), m_port(p_port) {
        p_itm.enableStimulusPort(m_port);
    }

    void
    putf(const char p_char) const {
        m_itm.sendPort(m_port, p_char);
    }

    static void
    putf(void *p_this, const char p_char) {
        const ItmPort *obj = reinterpret_cast<const ItmPort *>(p_this);
        obj->putf(p_char);
    }
};
/*****************************************************************************/

/*****************************************************************************/
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32_ITM_HPP_17543F25_1525_4FAC_8CDA_6691E88C7157 */
