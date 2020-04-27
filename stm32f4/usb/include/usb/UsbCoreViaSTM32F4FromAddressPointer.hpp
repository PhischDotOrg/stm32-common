/*-
 * $Copyright$
-*/
#ifndef _USBCOREVIASTM32F4_FROMADDRESSPOINTER_HPP_1E3ABF97_3BFF_4E83_8D5A_20CCA1C0DFC1
#define _USBCOREVIASTM32F4_FROMADDRESSPOINTER_HPP_1E3ABF97_3BFF_4E83_8D5A_20CCA1C0DFC1

#include <usb/UsbCoreViaSTM32F4.hpp>
#include <stm32f4xx.h>

/*******************************************************************************
 * This file is needed / used from <stm32f4/NvicViaSTM32F4.hpp -- in order to
 * break up a chicken-and-egg problem between the types, we need a forward
 * declaration here.
 ******************************************************************************/
namespace devices {
    class ScbViaSTM32F4;
    template<typename ScbT = devices::ScbViaSTM32F4> class NvicViaSTM32F4T;
}

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 * Forward declare and typedef the instance specific types for the USB cores
 ******************************************************************************/
template<intptr_t UsbT, typename GpioPinT = gpio::GpioPin,
  typename RccT = devices::RccViaSTM32F4, typename NvicT = devices::NvicViaSTM32F4T<> >
class UsbCoreViaSTM32F4FromAddressPointerT;

#if defined(USB_OTG_FS_PERIPH_BASE)
typedef UsbCoreViaSTM32F4FromAddressPointerT<USB_OTG_FS_PERIPH_BASE> UsbFullSpeedCore;
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
typedef UsbCoreViaSTM32F4FromAddressPointerT<USB_OTG_HS_PERIPH_BASE> UsbHighSpeedCore;
#endif

/*******************************************************************************
 * Helper Template to resolve the SPI Address to the corresponding GPIO
 * Alternate Function Defintion
 ******************************************************************************/
template<intptr_t> struct UsbCoreGpioFunction;

#if defined(USB_OTG_FS_PERIPH_BASE)
template<> struct UsbCoreGpioFunction<USB_OTG_FS_PERIPH_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_UsbFs;
};
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
template<> struct UsbCoreGpioFunction<USB_OTG_HS_PERIPH_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_UsbHs;
};
#endif

/*******************************************************************************
 * Helper Template to resolve the USB Core Base Address to the corresponding
 * RCC Function Defintion
 ******************************************************************************/
template<intptr_t> struct UsbCoreRccFunction;

#if defined(USB_OTG_FS_PERIPH_BASE)
template<> struct UsbCoreRccFunction<USB_OTG_FS_PERIPH_BASE> {
    static const devices::RccViaSTM32F4::FunctionAHB2_t m_type = devices::RccViaSTM32F4::e_OtgFs;
};
#endif
#if defined(USB_OTG_HS_PERIPH_BASE)
template<> struct UsbCoreRccFunction<USB_OTG_HS_PERIPH_BASE> {
    static const devices::RccViaSTM32F4::FunctionAHB1_t m_type = devices::RccViaSTM32F4::e_OtgHs;
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t UsbT, typename GpioPinT /* = gpio::GpioPin */, typename RccT /* = devices::RccViaSTM32F4 */, typename NvicT /* = devices::NvicViaSTM32F4*/>
class UsbCoreViaSTM32F4FromAddressPointerT : public UsbCoreViaSTM32F4 {
private:
    NvicT &     m_nvic;
    RccT &      m_rcc;
    GpioPinT &  m_pinDm;
    GpioPinT &  m_pinDp;
    GpioPinT &  m_pinVbus;
    GpioPinT &  m_pinId;
    
public:
    UsbCoreViaSTM32F4FromAddressPointerT(NvicT &p_nvic, RccT &p_rcc,
      GpioPinT &p_pinDm, GpioPinT &p_pinDp, GpioPinT &p_pinVbus, GpioPinT &p_pinId, uint32_t p_rxFifoSzInWords = 128);

    virtual ~UsbCoreViaSTM32F4FromAddressPointerT();
};

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#include <usb/UsbCoreViaSTM32F4FromAddressPointer.cpp>

#endif /* _USBCOREVIASTM32F4_FROMADDRESSPOINTER_HPP_1E3ABF97_3BFF_4E83_8D5A_20CCA1C0DFC1 */
