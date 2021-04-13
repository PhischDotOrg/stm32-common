/*-
 * $Copyright$
 */
#ifndef _STM32F1_PLLCFG_HPP_2288104F_8211_4374_8930_06F4AFD29889
#define _STM32F1_PLLCFG_HPP_2288104F_8211_4374_8930_06F4AFD29889

namespace stm32 {
    namespace f1 {
        struct PllCfg {
            static constexpr unsigned m_hsiSpeedInHz = 8 * 1000 * 1000;

            typedef enum PllSource_e : unsigned {
                e_PllSourceHSI = 0,
                e_PllSourceHSE = 1,
            } PllSource_t;

            typedef enum SysclkSource_e : unsigned {
                e_SysclkHSI = 0b00,
                e_SysclkHSE = 0b01,
                e_SysclkPLL = 0b10,
            } SysclkSource_t;

            typedef enum HSEPrescaler_e : unsigned {
                e_HSEPrescaler_None = 0,
                e_HSEPrescaler_Div2 = 1,
            } HSEPrescaler_t;

            typedef enum AHBPrescaler_e : unsigned {
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

            typedef enum APBPrescaler_e : unsigned {
                e_APBPrescaler_None     = 0b000,
                e_APBPrescaler_Div2     = 0b100,
                e_APBPrescaler_Div4     = 0b101,
                e_APBPrescaler_Div8     = 0b110,
                e_APBPrescaler_Div16    = 0b111,
            } APBPrescaler_t;

            typedef enum USBPrescaler_s : unsigned {
                e_USBPrescaler_Disabled = 0b00,
                e_USBPrescaler_None     = 0b10,
                e_USBPrescaler_Div15    = 0b11,
            } USBPrescaler_t;

            typedef enum PllMul_e : unsigned {
                e_PllM_2        = 0b0000,
                e_PllM_3        = 0b0001,
                e_PllM_4        = 0b0010,
                e_PllM_5        = 0b0011,
                e_PllM_6        = 0b0100,
                e_PllM_7        = 0b0101,
                e_PllM_8        = 0b0110,
                e_PllM_9        = 0b0111,
                e_PllM_10       = 0b1000,
                e_PllM_11       = 0b1001,
                e_PllM_12       = 0b1010,
                e_PllM_13       = 0b1011,
                e_PllM_14       = 0b1100,
                e_PllM_15       = 0b1101,
                e_PllM_16       = 0b1110,
            } PllMul_t;

            PllSource_t         m_pllSource;
            unsigned            m_hseSpeedInHz;
            HSEPrescaler_t      m_hsePrescaler;
            PllMul_t            m_pllM;
            SysclkSource_t      m_sysclkSource;
            AHBPrescaler_t      m_ahbPrescaler;
            APBPrescaler_t      m_apb1Prescaler;
            APBPrescaler_t      m_apb2Prescaler;
            USBPrescaler_t      m_usbPrescaler;

            constexpr SysclkSource_t getSysclkSource(void) const {
                return m_sysclkSource;
            }

            constexpr unsigned getPllInputSpeedInHz(void) const {
                return m_pllSource == PllSource_t::e_PllSourceHSI ? m_hsiSpeedInHz / 2
                  : m_pllSource == PllSource_t::e_PllSourceHSE ? (
                      m_hsePrescaler == HSEPrescaler_t::e_HSEPrescaler_None ? m_hseSpeedInHz
                    : m_hsePrescaler == HSEPrescaler_t::e_HSEPrescaler_Div2 ? m_hseSpeedInHz / 2
                    : 0
                  ) : 0;
            }

            constexpr unsigned getPllSpeedInHz(void) const {
                return getPllInputSpeedInHz() * (
                    m_pllM == PllMul_t::e_PllM_2  ? 2
                  : m_pllM == PllMul_t::e_PllM_3  ? 3
                  : m_pllM == PllMul_t::e_PllM_4  ? 4
                  : m_pllM == PllMul_t::e_PllM_5  ? 5
                  : m_pllM == PllMul_t::e_PllM_6  ? 6
                  : m_pllM == PllMul_t::e_PllM_7  ? 7
                  : m_pllM == PllMul_t::e_PllM_8  ? 8
                  : m_pllM == PllMul_t::e_PllM_9  ? 9
                  : m_pllM == PllMul_t::e_PllM_10 ? 10
                  : m_pllM == PllMul_t::e_PllM_11 ? 11
                  : m_pllM == PllMul_t::e_PllM_12 ? 12
                  : m_pllM == PllMul_t::e_PllM_13 ? 13
                  : m_pllM == PllMul_t::e_PllM_14 ? 14
                  : m_pllM == PllMul_t::e_PllM_15 ? 15
                  : m_pllM == PllMul_t::e_PllM_16 ? 16
                  : 0
                );
            }

            constexpr unsigned getSysclkSpeedInHz(void) const {
                return m_sysclkSource == SysclkSource_t::e_SysclkHSI ? m_hsiSpeedInHz
                  : m_sysclkSource == SysclkSource_t::e_SysclkHSE ? m_hseSpeedInHz
                  : m_sysclkSource == SysclkSource_t::e_SysclkPLL ? getPllSpeedInHz()
                  : 0;
            }

            constexpr unsigned getAhbSpeedInHz(void) const {
                return m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_None   ? getSysclkSpeedInHz() / 1
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div2   ? getSysclkSpeedInHz() / 2
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div4   ? getSysclkSpeedInHz() / 4
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div8   ? getSysclkSpeedInHz() / 8
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div16  ? getSysclkSpeedInHz() / 16
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div64  ? getSysclkSpeedInHz() / 64
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div128 ? getSysclkSpeedInHz() / 128
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div256 ? getSysclkSpeedInHz() / 256
                     : m_ahbPrescaler == AHBPrescaler_t::e_AHBPrescaler_Div512 ? getSysclkSpeedInHz() / 512
                     : 0;
            }

            constexpr unsigned getApb1SpeedInHz(void) const {
                return m_apb1Prescaler == APBPrescaler_t::e_APBPrescaler_None  ? getAhbSpeedInHz() / 1
                     : m_apb1Prescaler == APBPrescaler_t::e_APBPrescaler_Div2  ? getAhbSpeedInHz() / 2
                     : m_apb1Prescaler == APBPrescaler_t::e_APBPrescaler_Div4  ? getAhbSpeedInHz() / 4
                     : m_apb1Prescaler == APBPrescaler_t::e_APBPrescaler_Div8  ? getAhbSpeedInHz() / 8
                     : m_apb1Prescaler == APBPrescaler_t::e_APBPrescaler_Div16 ? getAhbSpeedInHz() / 16
                     : 0;
            }

            constexpr unsigned getApb2SpeedInHz(void) const {
                return m_apb2Prescaler == APBPrescaler_t::e_APBPrescaler_None  ? getAhbSpeedInHz() / 1
                     : m_apb2Prescaler == APBPrescaler_t::e_APBPrescaler_Div2  ? getAhbSpeedInHz() / 2
                     : m_apb2Prescaler == APBPrescaler_t::e_APBPrescaler_Div4  ? getAhbSpeedInHz() / 4
                     : m_apb2Prescaler == APBPrescaler_t::e_APBPrescaler_Div8  ? getAhbSpeedInHz() / 8
                     : m_apb2Prescaler == APBPrescaler_t::e_APBPrescaler_Div16 ? getAhbSpeedInHz() / 16
                     : 0;
            }

            constexpr unsigned getUsbSpeedInHz(void) const {
                return m_usbPrescaler == USBPrescaler_t::e_USBPrescaler_None ? getPllSpeedInHz() / 1
                     : m_usbPrescaler == USBPrescaler_t::e_USBPrescaler_Div15 ? (getPllSpeedInHz() * 2) / 3
                     : 0;
            }

            constexpr bool isValid(void) const {
                const bool isValid = true
                    && ((m_hseSpeedInHz == 0) || ((m_hseSpeedInHz >= 4 * 1000 * 1000) && (m_hseSpeedInHz <= 16 * 1000 * 1000)))
                    && (getPllInputSpeedInHz() >= 4 * 1000 * 1000) && (getPllInputSpeedInHz() <= 16 * 1000 * 1000)
                    && ((getSysclkSpeedInHz() >= 4 * 1000 * 1000) && (getSysclkSpeedInHz() <= 72 * 1000 * 1000))
                    && ((getAhbSpeedInHz() != 0) && (getAhbSpeedInHz() <= 72 * 1000 * 1000))
                    && ((getApb1SpeedInHz() != 0) && (getApb1SpeedInHz() <= 36 * 1000 * 1000))
                    && ((getApb2SpeedInHz() != 0) && (getApb2SpeedInHz() <= 72 * 1000 * 1000))
                    && ((m_usbPrescaler == USBPrescaler_t::e_USBPrescaler_Disabled) || (getUsbSpeedInHz() == 48 * 1000 * 1000))
                    && true;

                return isValid;
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
        };
    } /* namespace f1 */
} /* namespace stm32 */

#endif /* _STM32F1_PLLCFG_HPP_2288104F_8211_4374_8930_06F4AFD29889 */
