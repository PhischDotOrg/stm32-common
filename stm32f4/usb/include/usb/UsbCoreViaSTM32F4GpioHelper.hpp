/*-
 * $Copyright$
-*/

#ifndef _USBCOREVIASTM32F4GPIOHELPER_HPP_24D74B7D_744C_498B_A6CB_BDB3BEEFD456
#define _USBCOREVIASTM32F4GPIOHELPER_HPP_24D74B7D_744C_498B_A6CB_BDB3BEEFD456

#include <gpio/GpioAccessViaSTM32F4.hpp>

namespace usb {
    namespace stm32f4 {

/*******************************************************************************
 *
 ******************************************************************************/
template<gpio::GpioAccessViaSTM32F4::Function_e GpioFunctionT>
class UsbCoreViaSTM32F4GpioHelperT {
public:
    static void initalize(gpio::GpioPin &p_pinDm, gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId);
    static void terminate(gpio::GpioPin &p_pinDm, gpio::GpioPin &p_pinDp, gpio::GpioPin &p_pinVbus, gpio::GpioPin &p_pinId);
};

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */

#include <usb/UsbCoreViaSTM32F4GpioHelper.cpp>

#endif /* _USBCOREVIASTM32F4GPIOHELPER_HPP_24D74B7D_744C_498B_A6CB_BDB3BEEFD456 */