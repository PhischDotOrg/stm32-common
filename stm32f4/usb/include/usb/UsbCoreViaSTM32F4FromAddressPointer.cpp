/*-
 * $Copyright$
-*/

#include <usb/UsbCoreViaSTM32F4FromAddressPointer.hpp>
#include <usb/UsbCoreViaSTM32F4GpioHelper.hpp>

namespace usb {
    namespace stm32f4 {

template<intptr_t UsbT, typename GpioPinT, typename RccT, typename NvicT>
UsbCoreViaSTM32F4FromAddressPointerT<UsbT, GpioPinT, RccT, NvicT>::UsbCoreViaSTM32F4FromAddressPointerT(NvicT &p_nvic,
  RccT &p_rcc, GpioPinT &p_pinDm, GpioPinT &p_pinDp, GpioPinT &p_pinVbus, GpioPinT &p_pinId, uint32_t p_rxFifoSzInWords /* = 128 */)
    : UsbCoreViaSTM32F4(reinterpret_cast<USB_OTG_GlobalTypeDef *>(UsbT), UsbT + USB_OTG_PCGCCTL_BASE, p_rxFifoSzInWords), m_nvic(p_nvic),
        m_rcc(p_rcc), m_pinDm(p_pinDm), m_pinDp(p_pinDp), m_pinVbus(p_pinVbus), m_pinId(p_pinId)
{
    m_rcc.enable(UsbCoreRccFunction<UsbT>::m_type);
    UsbCoreViaSTM32F4GpioHelperT< UsbCoreGpioFunction<UsbT>::m_type >::initalize(m_pinDm, m_pinDp, m_pinVbus, m_pinId);
    this->initialize();
    m_nvic.enableIrq(*this);
}

/*******************************************************************************
 *
 *******************************************************************************/
template<intptr_t UsbT, typename GpioPinT, typename RccT, typename NvicT>
UsbCoreViaSTM32F4FromAddressPointerT<UsbT, GpioPinT, RccT, NvicT>::~UsbCoreViaSTM32F4FromAddressPointerT() {
    m_nvic.disableIrq(*this);
    this->terminate();
    UsbCoreViaSTM32F4GpioHelperT< UsbCoreGpioFunction<UsbT>::m_type >::terminate(m_pinDm, m_pinDp, m_pinVbus, m_pinId);
    m_rcc.disable(UsbCoreRccFunction<UsbT>::m_type);
}

/*******************************************************************************
 *
 *******************************************************************************/
    } /* namespace stm32f4 */
} /* namespace usb */
