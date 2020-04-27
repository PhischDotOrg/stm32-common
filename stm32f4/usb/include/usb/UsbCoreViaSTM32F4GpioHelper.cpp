/*-
 * $Copyright$
-*/

#include <usb/UsbCoreViaSTM32F4GpioHelper.hpp>
#include <gpio/GpioAccessViaSTM32F4.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<gpio::GpioAccessViaSTM32F4::Function_e GpioFunctionT>
void
UsbCoreViaSTM32F4GpioHelperT<GpioFunctionT>::initalize(gpio::GpioPin &p_pinDm,
  gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId) {
    p_pinDm.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
    p_pinDp.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
    p_pinVbus.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
    p_pinId.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
}

/*******************************************************************************
 *
 *******************************************************************************/
template<gpio::GpioAccessViaSTM32F4::Function_e GpioFunctionT>
void
UsbCoreViaSTM32F4GpioHelperT<GpioFunctionT>::terminate(gpio::GpioPin &p_pinDm,
  gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId) {
    p_pinDm.disable();
    p_pinDp.disable();
    p_pinVbus.disable();
    p_pinId.disable();
}

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */
