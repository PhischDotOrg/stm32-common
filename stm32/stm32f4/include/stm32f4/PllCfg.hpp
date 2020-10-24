/*-
 * $Copyright$
 */
#ifndef _STM32F4_PLLCFG_HPP_2F7F94A9_4DCF_4ADC_A13F_1A8B4C837F2D
#define _STM32F4_PLLCFG_HPP_2F7F94A9_4DCF_4ADC_A13F_1A8B4C837F2D

namespace stm32 {
    namespace f4 {
        template<
            unsigned nHsiSpeedInMhz,
            typename PllCfgValidCheckT
        >
        struct PllCfgT {
            typedef enum {
                e_SysclkHSI = 0,
                e_SysclkHSE = 1,
                e_SysclkPLL = 2,
                e_SysclkInvalid = 3,
            } SysclkSource_t;

            typedef enum {
                e_PllSourceHSI = 0,
                e_PllSourceHSE = 1,
            } PllSource_t;

            typedef enum {
                e_PllP_Div2 = 0,
                e_PllP_Div4 = 1,
                e_PllP_Div6 = 2,
                e_PllP_Div8 = 3,
            } PllP_t;

            typedef enum {
                e_PllQ_Div2     = 2,
                e_PllQ_Div3     = 3,
                e_PllQ_Div4     = 4,
                e_PllQ_Div5     = 5,
                e_PllQ_Div6     = 6,
                e_PllQ_Div7     = 7,
                e_PllQ_Div8     = 8,
                e_PllQ_Div9     = 9,
                e_PllQ_Div10    = 10,
                e_PllQ_Div11    = 11,
                e_PllQ_Div12    = 12,
                e_PllQ_Div13    = 13,
                e_PllQ_Div14    = 14,
                e_PllQ_Div15    = 15,
                e_PllQ_Disabled
            } PllQ_t;

            typedef enum : uint8_t {
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

            typedef enum : uint8_t {
                e_APBPrescaler_None     = 0b000,
                e_APBPrescaler_Div2     = 0b100,
                e_APBPrescaler_Div4     = 0b101,
                e_APBPrescaler_Div8     = 0b110,
                e_APBPrescaler_Div16    = 0b111,
            } APBPrescaler_t;

            static constexpr unsigned   m_hsiSpeedInHz = nHsiSpeedInMhz * 1000 * 1000;
            const PllSource_t           m_pllSource;
            const unsigned              m_hseSpeedInHz;
            const unsigned              m_pllM;
            const unsigned              m_pllN;
            const PllP_t                m_pllP;
            const PllQ_t                m_pllQ;
            const SysclkSource_t        m_sysclkSource;
            const AHBPrescaler_t        m_ahbPrescaler;
            const APBPrescaler_t        m_apb1Prescaler;
            const APBPrescaler_t        m_apb2Prescaler;

            constexpr SysclkSource_t getSysclkSource(void) const {
                return m_sysclkSource;
            }

            constexpr unsigned getPllInputSpeedInHz(void) const {
                return (
                      (m_pllSource == PllSource_t::e_PllSourceHSI) ? m_hsiSpeedInHz
                    : (m_pllSource == PllSource_t::e_PllSourceHSE) ? m_hseSpeedInHz
                    : 0
                );
            }

            constexpr bool isValid(void) const {
                return PllCfgValidCheckT::isValid(*this);
            }

            constexpr uint32_t getPllCfgReg(void) const {
                const uint32_t pllCfgReg = (
                      ((m_pllQ << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk)
                    | ((m_pllP << RCC_PLLCFGR_PLLP_Pos) & RCC_PLLCFGR_PLLP_Msk)
                    | ((m_pllN <<  RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk)
                    | ((m_pllM <<  RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk)
                    | ((m_pllSource == PllSource_t::e_PllSourceHSE) ? RCC_PLLCFGR_PLLSRC_HSE : RCC_PLLCFGR_PLLSRC_HSI)
                );

                return pllCfgReg;
            }

            constexpr bool enableHSI(void) const {
                return (m_pllSource == PllSource_t::e_PllSourceHSI) || (m_sysclkSource == SysclkSource_t::e_SysclkHSI);
            }

            constexpr bool enableHSE(void) const {
                return (m_pllSource == PllSource_t::e_PllSourceHSE) || (m_sysclkSource == SysclkSource_t::e_SysclkHSE);
            }

            constexpr bool enablePLL(void) const {
                return (m_sysclkSource == SysclkSource_t::e_SysclkPLL);
            }

            constexpr unsigned getSysclkSpeedInHz(void) const {
                const SysclkSource_t sysclkSrc = getSysclkSource();

                const unsigned speed =
                    sysclkSrc == SysclkSource_t::e_SysclkHSI ? m_hsiSpeedInHz
                    : sysclkSrc == SysclkSource_t::e_SysclkHSE ? m_hseSpeedInHz
                        : sysclkSrc == SysclkSource_t::e_SysclkPLL ? getPllSpeedInHz()
                        : 0;

                return speed;
            }

            constexpr unsigned getAhbSpeedInHz(void) const {
                const unsigned prescaler = getAHBPrescalerValue(m_ahbPrescaler);

                const unsigned ahbSpeedInHz = (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;

                return ahbSpeedInHz;
            }

            constexpr unsigned getApb1SpeedInHz(void) const {
                const unsigned prescaler = getAPBPrescalerValue(m_apb1Prescaler);

                return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;
            }

            constexpr unsigned getApb2SpeedInHz(void) const {
                unsigned prescaler = getAPBPrescalerValue(m_apb2Prescaler);

                return (getSysclkSpeedInHz() + (prescaler / 2)) / prescaler;    
            }

        protected:
            constexpr unsigned getPllSpeedInHz(void) const {
                unsigned vco = getPllVcoSpeedInHz();

                unsigned pllp = 2 << m_pllP;

                return (vco + (pllp / 2)) / pllp;
            }

            constexpr unsigned getPllVcoSpeedInHz(void) const {
                const unsigned input = getPllInputSpeedInHz();

                const unsigned vco = ((input * m_pllN) + (m_pllM / 2)) / m_pllM;

                return vco;
            }

            /*
             * Facade for the non-static getPllVcoSpeedInHz() to be used from the
             * PllCfgValidCheckT policy class.
             */
            static constexpr unsigned getPllVcoSpeedInHz(const PllCfgT &p_obj) {
                return p_obj.getPllVcoSpeedInHz();
            }

            constexpr unsigned getAPBPrescalerValue(const APBPrescaler_t p_prescaler) const {
                const unsigned value = (
                    APBPrescaler_t::e_APBPrescaler_None   == p_prescaler ? 1
                : APBPrescaler_t::e_APBPrescaler_Div2   == p_prescaler ? 2
                : APBPrescaler_t::e_APBPrescaler_Div4   == p_prescaler ? 4
                : APBPrescaler_t::e_APBPrescaler_Div8   == p_prescaler ? 8
                : APBPrescaler_t::e_APBPrescaler_Div16  == p_prescaler ? 16
                : 0
                );

                return value;        
            }
            
            static constexpr unsigned getAHBPrescalerValue(const AHBPrescaler_t p_prescaler) {
                const unsigned value = (
                    AHBPrescaler_t::e_AHBPrescaler_None   == p_prescaler ? 1
                : AHBPrescaler_t::e_AHBPrescaler_Div2   == p_prescaler ? 2
                : AHBPrescaler_t::e_AHBPrescaler_Div4   == p_prescaler ? 4
                : AHBPrescaler_t::e_AHBPrescaler_Div8   == p_prescaler ? 8
                : AHBPrescaler_t::e_AHBPrescaler_Div16  == p_prescaler ? 16
                : AHBPrescaler_t::e_AHBPrescaler_Div64  == p_prescaler ? 64
                : AHBPrescaler_t::e_AHBPrescaler_Div128 == p_prescaler ? 128
                : AHBPrescaler_t::e_AHBPrescaler_Div256 == p_prescaler ? 256
                : AHBPrescaler_t::e_AHBPrescaler_Div512 == p_prescaler ? 512
                : 0
                );

                return value;        
            }
        }; /* struct PllCfg */

        namespace f401 {
            struct PllCfgValidCheck {
                template<typename PllCfgT>
                static constexpr
                bool isValid(const PllCfgT &p_pllCfg) {
                    const bool isValid = true
                        && (p_pllCfg.m_pllN >= 50) && (p_pllCfg.m_pllN <= 432)
                        && (p_pllCfg.m_pllM >= 2) && (p_pllCfg.m_pllM <= 63)
                        // && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) >= 100 * 1000 * 1000) && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) <= 432 * 1000 * 1000)
                        && ((p_pllCfg.m_hseSpeedInHz >= 4 * 1000 * 1000) && (p_pllCfg.m_hseSpeedInHz <= 26 * 1000 * 1000))
                        && (p_pllCfg.getSysclkSpeedInHz() > 0) && (p_pllCfg.getSysclkSpeedInHz() <= 84 * 1000 * 1000)
                        && (p_pllCfg.getAhbSpeedInHz() > 0) && (p_pllCfg.getAhbSpeedInHz() <= p_pllCfg.getSysclkSpeedInHz())
                        && (p_pllCfg.getApb1SpeedInHz() > 0) && (p_pllCfg.getApb1SpeedInHz() <= 42 * 1000 * 1000)
                        && (p_pllCfg.getApb2SpeedInHz() > 0) && (p_pllCfg.getApb2SpeedInHz() <= 84 * 1000 * 1000)
                        && true;

                    return isValid;
                }                
            }; /* PllCfgValidCheck */
        } /* namespace f407 */

        namespace f407 {
            struct PllCfgValidCheck {
                template<typename PllCfgT>
                static constexpr
                bool isValid(const PllCfgT &p_pllCfg) {
                    const bool isValid = true
                        && (p_pllCfg.getPllInputSpeedInHz() >= 4 * 1000 * 1000) && (p_pllCfg.getPllInputSpeedInHz() <= 48 * 1000 * 1000)
                        && (p_pllCfg.m_pllN > 1) && (p_pllCfg.m_pllN < 433)
                        && (p_pllCfg.m_pllM > 1) && (p_pllCfg.m_pllM < 64)
                        // && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) >= 100 * 1000 * 1000) && (p_pllCfg.getPllVcoSpeedInHz(p_pllCfg) <= 432 * 1000 * 1000)
                        && ((p_pllCfg.m_hseSpeedInHz >= 4 * 1000 * 1000) && (p_pllCfg.m_hseSpeedInHz <= 48 * 1000 * 1000))
                        && (p_pllCfg.getSysclkSpeedInHz() > 0) && (p_pllCfg.getSysclkSpeedInHz() <= 168 * 1000 * 1000)
                        && (p_pllCfg.getAhbSpeedInHz() > 0) && (p_pllCfg.getAhbSpeedInHz() >= p_pllCfg.getSysclkSpeedInHz())
                        && true;

                    return isValid;
                }                
            }; /* PllCfgValidCheck */
        } /* namespace f407 */

        namespace f411 {
            struct PllCfgValidCheck {
                template<typename PllCfgT>
                static constexpr
                bool isValid(const PllCfgT &p_pllCfg) {
                    const bool isValid = true
                        && (p_pllCfg.getPllInputSpeedInHz() >= 4 * 1000 * 1000) && (p_pllCfg.getPllInputSpeedInHz() <= 48 * 1000 * 1000)
                        && (p_pllCfg.m_pllN >= 50) && (p_pllCfg.m_pllN <= 432)
                        && (p_pllCfg.m_pllM >= 2) && (p_pllCfg.m_pllM <= 63)
                        // && (getPllVcoSpeedInHz(p_pllCfg) >= 100 * 1000 * 1000) && (getPllVcoSpeedInHz(p_pllCfg) <= 432 * 1000 * 1000)
                        && (((p_pllCfg.m_hseSpeedInHz >= 4 * 1000 * 1000) && (p_pllCfg.m_hseSpeedInHz <= 48 * 1000 * 1000)) || p_pllCfg.m_hseSpeedInHz == 0)
                        && (p_pllCfg.getSysclkSpeedInHz() > 0) && (p_pllCfg.getSysclkSpeedInHz() <= 100 * 1000 * 1000)
                        && (p_pllCfg.getAhbSpeedInHz() > 0) && (p_pllCfg.getAhbSpeedInHz() >= p_pllCfg.getSysclkSpeedInHz())
                        && true;

                    return isValid;
                }                
            }; /* PllCfgValidCheck */
        } /* namespace f411 */
    } /* namespace f4 */
} /* namespace stm32 */

#endif /* _STM32F4_PLLCFG_HPP_2F7F94A9_4DCF_4ADC_A13F_1A8B4C837F2D */
