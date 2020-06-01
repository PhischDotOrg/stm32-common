/*-
 * $Copyright$
-*/

#include <usb/UsbCoreViaSTM32F4.hpp>

extern usb::stm32f4::UsbFullSpeedCore usbCore;

/*******************************************************************************
 *
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */

void
OTG_FS_WKUP_IRQHandler(void) {
    while (1) ;
}

void
OTG_FS_IRQHandler(void) {
    usbCore.handleIrq();
}

void
OTG_HS_EP1_OUT_IRQHandler(void) {
    while (1) ;
}

void
OTG_HS_EP1_IN_IRQHandler(void) {
    while (1) ;
}

void
OTG_HS_WKUP_IRQHandler(void) {
    while (1) ;
}

void
OTG_HS_IRQHandler(void) {
    while (1) ;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined (__cplusplus) */