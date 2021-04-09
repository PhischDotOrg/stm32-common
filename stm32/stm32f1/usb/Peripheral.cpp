/*-
 * $Copyright$
-*/

#include "usb/Peripheral.hpp"
#include "usb/Device.hpp"
#include <usb/UsbTypes.hpp>

#include "f1usb/src/usbutils.hh"


/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

void
Peripheral::enterPwrDown(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_PDWN;
}

void
Peripheral::exitPwrDown(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_PDWN;
}

void
Peripheral::enterLowPowerMode(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_LP_MODE;
}

void
Peripheral::exitLowPowerMode(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_LP_MODE;
}

void
Peripheral::forceReset(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_FRES;
}

void
Peripheral::clearReset(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_FRES;
}

void
Peripheral::start(void) const {
    USB_PRINTF("--> Peripheral::%s()\r\n", __func__);

    exitLowPowerMode();
    exitPwrDown();

    delay();

    clearInterrupt(Interrupt_e::e_All);

    enableInterrupts();

    clearReset();

    enableFunction();

    USB_PRINTF("<-- Peripheral::%s()\r\n", __func__);
}

void
Peripheral::stop(void) const {
    USB_PRINTF("--> Peripheral::%s()\r\n", __func__);

    disableFunction();

    disableInterrupts();

    forceReset();
    enterPwrDown();
    enterLowPowerMode();

    USB_PRINTF("<-- Peripheral::%s()\r\n", __func__);
}

void
Peripheral::clearInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.ISTR = static_cast<uint16_t>(Interrupt_t::e_All) ^ static_cast<uint16_t>(p_irq);
}

void
Peripheral::enableInterrupts(void) const {
    for (const Peripheral::irq_handler_t *cur = Peripheral::m_irq_handler; cur->m_irq != Interrupt_e::e_None; cur++) {
        enableInterrupt(cur->m_irq);
    }
}

void
Peripheral::enableInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.CNTR |= static_cast<uint16_t>(p_irq);
}

void
Peripheral::disableInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.CNTR |= ~(static_cast<uint16_t>(p_irq));
}

const
Peripheral::irq_handler_t Peripheral::m_irq_handler[] = {
    { Interrupt_e::e_Error,             &Peripheral::handleErrorIrq },
    { Interrupt_e::e_Reset,             &Peripheral::handleResetIrq },
    { Interrupt_e::e_CorrectTransfer,   &Peripheral::handleCorrectTransferIrq },
    { Interrupt_e::e_None,              nullptr }
};

void
Peripheral::handleIrq(void) const {
    uint16_t irqStatus;
    uint16_t handledIrq = 0;
    
    irqStatus = this->m_usbCore.ISTR & 0xFF00u;

    USB_PRINTF("--> Peripheral::%s(irqStatus=0x%x)\r\n", __func__, irqStatus);

    for (const Peripheral::irq_handler_t *cur = Peripheral::m_irq_handler; cur->m_irq != Interrupt_e::e_None; cur++) {
        if (irqStatus & static_cast<uint16_t>(cur->m_irq)) {
            handledIrq |= static_cast<uint16_t>(cur->m_irq);
            (this->*(cur->m_fn))(); // Call member function via pointer
            clearInterrupt(cur->m_irq);
        }
    }

    USB_PRINTF("<-- Peripheral::%s(): handled IRQ=0x%x (ignored=0x%x)\r\n\r\n", __func__, handledIrq, irqStatus & ~handledIrq);
}

void
Peripheral::handleResetIrq(void) const {
    USB_PRINTF("--> Peripheral::%s()\r\n", __func__);

    clearReset();

    if (this->m_device != nullptr) {
        m_device->reset();
    }

    USB_PRINTF("<-- Peripheral::%s()\r\n", __func__);
}

void
Peripheral::handleCorrectTransferIrq(void) const {
    USB_PRINTF("--> Peripheral::%s()\r\n", __func__);

    unsigned direction = (m_usbCore.ISTR >> USB_ISTR_DIR_Pos) & 0b1;
    unsigned endpointNo = (m_usbCore.ISTR >> USB_ISTR_EP_ID_Pos) & USB_ISTR_EP_ID_Msk;

    if (this->m_device != nullptr) {
        m_device->handleEndpointIrq(endpointNo, direction);
    }

    USB_PRINTF("<-- Peripheral::%s()\r\n", __func__);
}

void
Peripheral::handleErrorIrq(void) const {
    USB_PRINTF("--> Peripheral::%s()\r\n", __func__);

    __BKPT();

    USB_PRINTF("<-- Peripheral::%s()\r\n", __func__);
}

void
Peripheral::setAddress(uint8_t p_address) const {
    USB_PRINTF("--> Peripheral::%s(p_address=%d (0x%x))\r\n", __func__, p_address, p_address);

    static constexpr uint16_t mask = 0b0011'1111;

    this->m_usbCore.DADDR &= ~mask;
    this->m_usbCore.DADDR |= p_address & mask;

    USB_PRINTF("<-- Peripheral::%s(p_address=%d (0x%x))\r\n", __func__, p_address, p_address);
}

void
Peripheral::setBufferTable(const void * const p_bufferTable) const {
    const uintptr_t btableHostAddr = reinterpret_cast<uintptr_t>(p_bufferTable);
    const uintptr_t btablePeriphAddr = mapHostToPeripheral(btableHostAddr);

    assert((btableHostAddr & 0b111) == 0);
    assert((btablePeriphAddr & 0b111) == 0);

    this->m_usbCore.BTABLE = btablePeriphAddr;
}

void
Peripheral::enableFunction(void) const {
    this->m_usbCore.DADDR |= USB_DADDR_EF;
}

void
Peripheral::disableFunction(void) const {
    this->m_usbCore.DADDR &= ~USB_DADDR_EF;
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
