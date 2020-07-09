/*-
 * $Copyright$
-*/

#ifndef _GPIO_ACCESS_VIA_STM32F4_HPP_b524ac1b_90b2_4060_99aa_666543cfff6a
#define _GPIO_ACCESS_VIA_STM32F4_HPP_b524ac1b_90b2_4060_99aa_666543cfff6a

#include <stdint.h>
#include <stddef.h>

#if defined(DEBUG_BUILD) && defined(GMOCK_FOUND)
#include <gmock/gmock.h>
#endif /* defined(DEBUG_BUILD) && defined(GMOCK_FOUND) */

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/RccViaSTM32.hpp>

namespace gpio {

/*******************************************************************************
 *
 ******************************************************************************/
class GpioAccessViaSTM32F4 {
public:
    static const size_t m_width = 16;

    enum Gpio_e {
        e_GpioA,
        e_GpioB,
        e_GpioC,
        e_GpioD,
        e_GpioE,
        e_GpioF,
        e_GpioG,
        e_GpioH,
        e_GpioI
    };

    enum Mode_e {
        e_Input = 0,
        e_Output = 1,
        e_Alternate = 2,
        e_Analog = 3
    };

    enum Termination_e {
        e_None = 0,
        e_PullUp = 1,
        e_PullDown = 2
    };
    
    typedef enum Function_e {
        e_Gpio = 0,
        e_Mco = 0,
        e_Tim1 = 1,
        e_Tim2 = 1,
        e_Tim3 = 2,
        e_Tim4 = 2,
        e_Tim5 = 2,
        e_Tim8 = 3,
        e_Tim9 = 3,
        e_Tim10 = 3,
        e_Tim11 = 3,
//        e_I2c1 = 1,
//        e_I2c2 = 1,
//        e_I2c3 = 1,
        e_Spi1 = 5,
        e_Spi2 = 5,
        e_Spi3 = 6,
        e_Uart1 = 7,
        e_Uart2 = 7,
        e_Uart3 = 7,
        e_Uart4 = 8,
        e_Uart5 = 8,
        e_Uart6 = 8,
        e_UsbFs = 10,
        e_UsbHs = 10,
        e_ForceTo32Bit = 0xffffffff,
    } Function_t;

    void write(uint16_t p_value, uint16_t p_output, uint16_t p_mask) const;
    void read(uint16_t &p_vector) const;

    int enable(const uint8_t p_pin, const Mode_e p_mode, const Termination_e p_termination, const Function_e p_function) const;
    int disable(const uint8_t p_pin) const;

protected:
    GpioAccessViaSTM32F4(GPIO_TypeDef * const p_gpio);
    ~GpioAccessViaSTM32F4(void);

private:
    GPIO_TypeDef * const m_gpio;

};

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t> struct GpioAccessViaSTM32F4RccFunction;

#if defined(GPIOA_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOA_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioA;
};
#endif

#if defined(GPIOB_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOB_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioB;
};
#endif

#if defined(GPIOC_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOC_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioC;
};
#endif

#if defined(GPIOD_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOD_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioD;
};
#endif

#if defined(GPIOE_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOE_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioE;
};
#endif

#if defined(GPIOF_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOF_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioF;
};
#endif

#if defined(GPIOG_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOG_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioG;
};
#endif

#if defined(GPIOH_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOH_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioH;
};
#endif

#if defined(GPIOI_BASE)
template<> struct GpioAccessViaSTM32F4RccFunction<GPIOI_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_GpioI;
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t GpioT, typename RccT = devices::RccViaSTM32F4>
class GpioAccessViaSTM32F4T : public GpioAccessViaSTM32F4 {

public:
    GpioAccessViaSTM32F4T(RccT &p_rcc) : GpioAccessViaSTM32F4(reinterpret_cast<GPIO_TypeDef *>(GpioT)), m_rcc(p_rcc) {
        m_rcc.enable(GpioAccessViaSTM32F4RccFunction<GpioT>::m_type);
    };

    ~GpioAccessViaSTM32F4T() {
        m_rcc.disable(GpioAccessViaSTM32F4RccFunction<GpioT>::m_type);
    }
    
private:
    RccT &m_rcc;
};

#if defined(GPIOA_BASE)
typedef GpioAccessViaSTM32F4T<GPIOA_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioA;
#endif
#if defined(GPIOB_BASE)
typedef GpioAccessViaSTM32F4T<GPIOB_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioB;
#endif
#if defined(GPIOC_BASE)
typedef GpioAccessViaSTM32F4T<GPIOC_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioC;
#endif
#if defined(GPIOD_BASE)
typedef GpioAccessViaSTM32F4T<GPIOD_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioD;
#endif
#if defined(GPIOE_BASE)
typedef GpioAccessViaSTM32F4T<GPIOE_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioE;
#endif
#if defined(GPIOF_BASE)
typedef GpioAccessViaSTM32F4T<GPIOF_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioF;
#endif
#if defined(GPIOG_BASE)
typedef GpioAccessViaSTM32F4T<GPIOG_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioG;
#endif
#if defined(GPIOH_BASE)
typedef GpioAccessViaSTM32F4T<GPIOH_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioH;
#endif
#if defined(GPIOI_BASE)
typedef GpioAccessViaSTM32F4T<GPIOI_BASE, devices::RccViaSTM32F4> GpioAccessViaSTM32F4_GpioI;
#endif

}; /* namespace gpio */

#endif /* !_GPIO_ACCESS_VIA_STM32F4_HPP_b524ac1b_90b2_4060_99aa_666543cfff6a */
