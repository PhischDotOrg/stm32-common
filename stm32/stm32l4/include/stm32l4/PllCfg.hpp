/*-
 * $Copyright$
 */
#ifndef _STM32L4_PLLCFG_HPP_C19B2F0F_86BD_43EE_AADE_D4DBE81DC78C
#define _STM32L4_PLLCFG_HPP_C19B2F0F_86BD_43EE_AADE_D4DBE81DC78C

/*****************************************************************************/
namespace stm32 {
    namespace l4 {
/*****************************************************************************/

template<
    typename PllCfgValidCheckT,
    unsigned nHsiSpeedInMhz = 16,
    unsigned nLsiSpeedInKhz = 32
>
struct PllCfgT {
    typedef enum {
        e_SysclkMSI = 0,
        e_SysclkHSI = 1,
        e_SysclkHSE = 2,
        e_SysclkPLL = 3,
        e_SysclkInvalid,
    } SysclkSource_t;

    typedef enum {
        e_PllSourceNone = 0,
        e_PllSourceMSI  = 1,
        e_PllSourceHSI  = 2,
        e_PllSourceHSE  = 3,
    } PllSource_t;

    typedef enum {
        e_PllR_Div2 = 0,
        e_PllR_Div4 = 1,
        e_PllR_Div6 = 2,
        e_PllR_Div8 = 3,
        e_PllR_Disabled
    } PllR_t;

    typedef enum {
        e_PllQ_Div2 = 0,
        e_PllQ_Div4 = 1,
        e_PllQ_Div6 = 2,
        e_PllQ_Div8 = 3,
        e_PllQ_Disabled
    } PllQ_t;

    typedef enum {
        e_PllP_Div7     = 0,
        e_PllP_Div17    = 1,
        e_PllP_Disabled
    } PllP_t;

    typedef enum {
        e_AHBPrescaler_None     = 0b0000,
        e_AHBPrescaler_Div2     = 0b1000,
        e_AHBPrescaler_Div4     = 0b1001,
        e_AHBPrescaler_Div8     = 0b1010,
        e_AHBPrescaler_Div16    = 0b1011,
        e_AHBPrescaler_Div64    = 0b1100,
        e_AHBPrescaler_Div128   = 0b1101,
        e_AHBPrescaler_Div256   = 0b1110,
        e_AHBPrescaler_Div512   = 0b1111,
    } AHBPrescaler_t;

    typedef enum {
        e_APBPrescaler_None     = 0b000,
        e_APBPrescaler_Div2     = 0b100,
        e_APBPrescaler_Div4     = 0b101,
        e_APBPrescaler_Div8     = 0b110,
        e_APBPrescaler_Div16    = 0b111,
    } APBPrescaler_t;

    typedef enum {
        e_MSIRange_100kHz       = 0,
        e_MSIRange_200kHz       = 1,
        e_MSIRange_400kHz       = 2,
        e_MSIRange_800kHz       = 3,
        e_MSIRange_1MHz         = 4,
        e_MSIRange_2MHz         = 5,
        e_MSIRange_4MHz         = 6,
        e_MSIRange_8MHz         = 7,
        e_MSIRange_16MHz        = 8,
        e_MSIRange_24MHz        = 9,
        e_MSIRange_32MHz        = 10,
        e_MSIRange_48MHz        = 11,
        e_MSIRange_Invalid
    } MSIRange_t;

    static const unsigned   m_hsiSpeedInHz = nHsiSpeedInMhz * 1000 * 1000;
    static const unsigned   m_lsiSpeedInHz = nLsiSpeedInKhz * 1000;

    const PllSource_t       m_pllSource;
    const MSIRange_t        m_msiRange;
    const unsigned          m_hseSpeedInHz;

    const unsigned          m_pllM;
    const unsigned          m_pllN;
    const PllP_t            m_pllP;
    const PllQ_t            m_pllQ;
    const PllR_t            m_pllR;

    const unsigned          m_pllSaiN;
    const PllP_t            m_pllSaiP;
    const PllQ_t            m_pllSaiQ;
    const PllR_t            m_pllSaiR;

    const SysclkSource_t    m_sysclkSource;
    const AHBPrescaler_t    m_ahbPrescaler;
    const APBPrescaler_t    m_apb1Prescaler;
    const APBPrescaler_t    m_apb2Prescaler;

    constexpr SysclkSource_t getSysclkSource(void) const {
        return m_sysclkSource;
    }

    constexpr unsigned getMSISpeedInHz(void) const {
        unsigned msiSpeedInHz =
            m_msiRange == MSIRange_t::e_MSIRange_100kHz ?   100 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_200kHz ?   200 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_400kHz ?   100 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_800kHz ?   200 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_1MHz   ?  1000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_2MHz   ?  2000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_4MHz   ?  4000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_8MHz   ?  8000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_16MHz  ? 16000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_24MHz  ? 24000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_32MHz  ? 32000 * 1000
          : m_msiRange == MSIRange_t::e_MSIRange_48MHz  ? 48000 * 1000
          : 0;

        return msiSpeedInHz;
    }

    constexpr unsigned getHSISpeedInHz(void) const {
        static_assert(m_hsiSpeedInHz == 16 * 1000 * 1000);

        return m_hsiSpeedInHz;
    }

    constexpr unsigned getHSESpeedInHz(void) const {
        return m_hseSpeedInHz;
    }

    constexpr unsigned getPllInputSpeedInHz(void) const {
        return (
          (m_pllSource == PllSource_t::e_PllSourceMSI) ? getMSISpeedInHz()
            : (m_pllSource == PllSource_t::e_PllSourceHSI) ? getHSISpeedInHz()
              : (m_pllSource == PllSource_t::e_PllSourceHSE) ? getHSESpeedInHz()
                : 0
        );

        // FIXME
        // static_assert(speed > 0);
        // static_assert(speed <= 48 * 1000 * 1000);

        // return speed;
    }

    constexpr unsigned getPllVcoSpeedInHz(void) const {
        const unsigned input = getPllInputSpeedInHz();
        // static_assert(input != 0);

        const unsigned plln = m_pllN;
        // static_assert((plln > 8) && (plln <= 86));

        const unsigned pllm = m_pllM;
        // static_assert(pllm > 0);

        const unsigned vco = ((input * plln) + (pllm / 2)) / pllm;
        // static_assert(vco != 0);

        return vco;
    }

    constexpr unsigned getPllSpeedInHz(void) const {
        const unsigned vco = getPllVcoSpeedInHz();

        const unsigned pllr = 2 << m_pllR;
     
        return (vco + (pllr / 2)) / pllr;
    }

    constexpr unsigned getSysclkSpeedInHz(void) const {
        SysclkSource_t sysclkSrc = getSysclkSource();

        unsigned speed =
            sysclkSrc == SysclkSource_t::e_SysclkMSI ? getMSISpeedInHz()
              : sysclkSrc == SysclkSource_t::e_SysclkHSI ? getHSISpeedInHz()
                : sysclkSrc == SysclkSource_t::e_SysclkHSE ? getHSESpeedInHz()
                  : sysclkSrc == SysclkSource_t::e_SysclkPLL ? getPllSpeedInHz()
                    : 0;

        return speed;
    }

    constexpr unsigned getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) const {
        unsigned value = 0;

        switch (p_prescaler) {
        case AHBPrescaler_t::e_AHBPrescaler_Div2:
        case AHBPrescaler_t::e_AHBPrescaler_Div4:
        case AHBPrescaler_t::e_AHBPrescaler_Div8:
        case AHBPrescaler_t::e_AHBPrescaler_Div16:
            value = 2 << p_prescaler;
            break;
        case AHBPrescaler_t::e_AHBPrescaler_Div64:
        case AHBPrescaler_t::e_AHBPrescaler_Div128:
        case AHBPrescaler_t::e_AHBPrescaler_Div256:
        case AHBPrescaler_t::e_AHBPrescaler_Div512:
            value = 4 << p_prescaler;
            break;
        case AHBPrescaler_t::e_AHBPrescaler_None:
            value = 1;
            break;
        }

        return value;        
    }

    constexpr unsigned getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const {
        const unsigned value = (p_prescaler == APBPrescaler_t::e_APBPrescaler_None ? 1 : 2 << p_prescaler);
        // static_assert(value != 0);

        return value;        
    }

    constexpr unsigned getAhbSpeedInHz(void) const {
        unsigned prescaler = getAHBPrescalerValue(m_ahbPrescaler);
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 512);

        unsigned ahbSpeedInHz = (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
        // FIXME
        // static_assert(ahbSpeedInHz > 0);
        // static_assert(ahbSpeedInHz <= 80 * 1000 * 1000);

        return ahbSpeedInHz;
    }

    constexpr unsigned getApb1SpeedInHz(void) const {
        unsigned prescaler = getAPBPrescalerValue(m_apb1Prescaler);
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 16);

        return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
    }

    constexpr unsigned getApb2SpeedInHz(void) const {
        unsigned prescaler = getAPBPrescalerValue(m_apb2Prescaler);
        // FIXME
        // static_assert(prescaler > 0);
        // static_assert(prescaler <= 16);

        return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;    
    }

     constexpr bool enableHSE(void) const {
        return (m_pllSource == PllSource_t::e_PllSourceHSE) || (m_sysclkSource == SysclkSource_t::e_SysclkHSE);
    }

    constexpr bool enablePLL(void) const {
        return (m_sysclkSource == SysclkSource_t::e_SysclkPLL);
    }

    constexpr bool enableHSI(void) const {
        return (m_pllSource == PllSource_t::e_PllSourceHSI) || (m_sysclkSource == SysclkSource_t::e_SysclkHSI);
    }

    constexpr bool enableMSI(void) const {
        return ((m_pllSource == PllSource_t::e_PllSourceMSI) || (m_sysclkSource == SysclkSource_t::e_SysclkMSI));
    }

    constexpr uint32_t getPllCfgReg(void) const {
        uint32_t pllCfgReg = (
                  (((m_pllM - 1) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk)
                | ((m_pllN << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk)

                | ((m_pllP << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk)
                | ((getPllPEnable() << RCC_PLLCFGR_PLLPEN_Pos) & RCC_PLLCFGR_PLLPEN_Msk)

                | ((m_pllQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk)
                | ((getPllQEnable() << RCC_PLLCFGR_PLLQEN_Pos) & RCC_PLLCFGR_PLLQEN_Msk)

                | ((m_pllR << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR_Msk)
                | ((getPllREnable() << RCC_PLLCFGR_PLLREN_Pos) & RCC_PLLCFGR_PLLREN_Msk)

                | ((m_pllSource << RCC_PLLCFGR_PLLSRC_Pos) & RCC_PLLCFGR_PLLSRC_Msk)
            );

        // static_assert(pllCfgReg != 0);

        return pllCfgReg;
    }

    constexpr bool isValid(void) const {
        return PllCfgValidCheckT::isValid(*this);
    }

private:
    constexpr bool getPllPEnable() const {
        return (m_pllP != PllP_t::e_PllP_Disabled);
    }

    constexpr bool getPllQEnable() const {
        return (m_pllQ != PllQ_t::e_PllQ_Disabled);
    }

    constexpr bool getPllREnable() const {
        return (m_pllR != PllR_t::e_PllR_Disabled);
    }
};

namespace l432 {
    struct PllCfgValidCheck {
        template<typename PllCfgT>
        static constexpr
        bool isValid(const PllCfgT & /* p_pllCfg */) {
            const bool isValid = true
                // && (p_pllCfg.getPllInputSpeedInHz() >= 4 * 1000 * 1000) && (p_pllCfg.getPllInputSpeedInHz() <= 48 * 1000 * 1000)
                // && (p_pllCfg.m_pllN > 1) && (p_pllCfg.m_pllN < 433)
                // && (p_pllCfg.m_pllM > 1) && (p_pllCfg.m_pllM < 64)
                // // && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) >= 100 * 1000 * 1000) && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) <= 432 * 1000 * 1000)
                // && ((p_pllCfg.m_hseSpeedInHz >= 4 * 1000 * 1000) && (p_pllCfg.m_hseSpeedInHz <= 48 * 1000 * 1000))
                // && (p_pllCfg.getSysclkSpeedInHz() > 0) && (p_pllCfg.getSysclkSpeedInHz() <= 168 * 1000 * 1000)
                // && (p_pllCfg.getAhbSpeedInHz() > 0) && (p_pllCfg.getAhbSpeedInHz() >= p_pllCfg.getSysclkSpeedInHz())
                && true;

            return isValid;
        }                
    }; /* PllCfgValidCheck */
} /* namespace l432 */

/*****************************************************************************/
    } /* namespace l4 */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _STM32L4_PLLCFG_HPP_C19B2F0F_86BD_43EE_AADE_D4DBE81DC78C */
