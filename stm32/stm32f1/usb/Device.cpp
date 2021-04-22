/*-
 * $Copyright$
-*/

#include "usb/Device.hpp"
#include "usb/InEndpoint.hpp"
#include "usb/OutEndpoint.hpp"
#include <usb/UsbTypes.hpp>

#include "f1usb/src/usbutils.hh"


/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

alignas(8) struct EndpointBufferDescriptor_s Device::m_bufferDescriptorTable[m_maxEndpoints];

void
Device::enterPwrDown(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_PDWN;
}

void
Device::exitPwrDown(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_PDWN;
}

void
Device::enterLowPowerMode(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_LP_MODE;
}

void
Device::exitLowPowerMode(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_LP_MODE;
}

void
Device::forceReset(void) const {
    this->m_usbCore.CNTR |= USB_CNTR_FRES;
}

void
Device::clearReset(void) const {
    this->m_usbCore.CNTR &= ~USB_CNTR_FRES;
}

void
Device::start(void) const {
    exitLowPowerMode();
    exitPwrDown();

    delay();

    clearInterrupt(Interrupt_e::e_All);

    enableInterrupts();

    clearReset();

    enableFunction();
}

void
Device::stop(void) const {
    disableFunction();

    disableInterrupts();

    forceReset();
    enterPwrDown();
    enterLowPowerMode();
}

void
Device::reset(void) const {
    /* Initialize Buffer Descriptor Table */
    for (unsigned idx = 0; idx < m_maxEndpoints; idx++) {
        m_bufferDescriptorTable[idx].m_txAddr   = 0;
        m_bufferDescriptorTable[idx].m_txCount  = 0;
        m_bufferDescriptorTable[idx].m_rxAddr   = 0;
        m_bufferDescriptorTable[idx].m_rxCount  = 0;
    }

    setAddress(0);

    for (unsigned idx = 0; idx < this->m_maxInEndpoints; idx++) {
        InEndpoint *inEp = m_inEndpoints[idx];
        OutEndpoint *outEp = m_outEndpoints[idx];

        /* Setup Endpoints
         * USB_EPRX_STAT    = USB_EP_RX_NAK (0b10)
         * USB_EP_T_FIELD   = Endpoint Type (Bulk, Ctrl, Iso, Irq)
         * USB_EPADDR_FIELD = Endpoint Address (as in USB Descriptor)
         * USB_EP_KIND      = 0b0
         * USB_EPTX_STAT    = USB_EP_TX_NAK (0b10)
         */
        if (inEp != nullptr) {
            inEp->reset();
        }

        if (outEp != nullptr) {
            outEp->reset();
        }
    }
 
    setBufferTable(this->m_bufferDescriptorTable);
}

void
Device::clearInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.ISTR = static_cast<uint16_t>(Interrupt_t::e_All) ^ static_cast<uint16_t>(p_irq);
}

void
Device::enableInterrupts(void) const {
    for (auto irq : Device::m_irq_handler) {
        enableInterrupt(irq.m_irq);
    }
}

void
Device::enableInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.CNTR |= static_cast<uint16_t>(p_irq);
}

void
Device::disableInterrupt(Interrupt_t p_irq) const {
    this->m_usbCore.CNTR |= ~(static_cast<uint16_t>(p_irq));
}

/* Recursively calculate IRQ mask at compile time */
constexpr uint16_t
Device::getIrqMask(uint16_t p_mask, irq_handler_table_t::const_iterator p_cur, irq_handler_table_t::const_iterator p_end) {
    return (p_cur != p_end) ? getIrqMask(p_mask | static_cast<uint16_t>(p_cur->m_irq), p_cur + 1, p_end) : p_mask;
}

void
Device::handleIrq(void) const {
    static constexpr uint16_t irq_mask = getIrqMask(0, m_irq_handler.begin(), m_irq_handler.end());
    static_assert(irq_mask == (USB_CNTR_ERRM | USB_CNTR_RESETM | USB_CNTR_CTRM | USB_CNTR_PMAOVRM)); // Ensure getIrqMask() is actually calculated at compile time

    const uint16_t irqStatus = m_usbCore.ISTR;

    if (irqStatus & irq_mask) {
        for (auto cur : Device::m_irq_handler) {
            if (irqStatus & static_cast<uint16_t>(cur.m_irq)) {
                (this->*(cur.m_fn))(irqStatus); // Call member function via pointer
            }
        }

        m_usbCore.ISTR &= ~irq_mask;
    }
}

void
Device::handleResetIrq(const uint16_t /* p_istr */) const {
    clearReset();

    this->reset();
}

void
Device::handleCorrectTransferIrq(const uint16_t p_istr) const {
    unsigned direction = (p_istr & USB_ISTR_DIR_Msk) >> USB_ISTR_DIR_Pos;
    unsigned endpointNo = (p_istr & USB_ISTR_EP_ID_Msk) >> USB_ISTR_EP_ID_Pos;

    this->handleEndpointIrq(endpointNo, direction);
}

void
Device::handleErrorIrq(const uint16_t /* p_istr */) const {
    assert(false);
}

void
Device::handlePacketMemOverrunIrq(const uint16_t /* p_istr */) const {
    assert(false);
}

void
Device::handleEndpointIrq(unsigned p_endpointNo, unsigned p_direction) const {
    assert(p_endpointNo < this->m_maxEndpoints);
    assert(p_endpointNo < this->m_maxInEndpoints);
    assert(p_endpointNo < this->m_maxOutEndpoints);

    const InEndpoint * const inEndp = m_inEndpoints[p_endpointNo];
    const OutEndpoint * const outEndp = m_outEndpoints[p_endpointNo];
    CtrlOutEndpoint * const ctrlOutEndp = m_ctrlOutEndpoint;

    const unsigned irqStatus = getHwEndpt(p_endpointNo).EPxR;

    /* TODO Is this really the right order? */
    if (inEndp != nullptr) {
        inEndp->handleIrq(irqStatus);
    }

    if (p_direction != 0) {
        if (p_endpointNo == 0) {
            assert(ctrlOutEndp != nullptr);
            ctrlOutEndp->handleIrq(irqStatus);
        } else if (outEndp != nullptr) {
            assert(outEndp != nullptr);
            outEndp->handleIrq(irqStatus);
        }
    }
}

void
Device::setAddress(uint8_t p_address) const {
    assert(this->m_inEndpoints[0] != nullptr);
    PHISCH_SETPIN(4, p_address != 0);

    static constexpr uint16_t mask = 0b0011'1111;
    this->m_usbCore.DADDR = (USB_DADDR_EF | (p_address & mask));
}

void
Device::setBufferTable(const void * const p_bufferTable) const {
    const uint16_t btablePeriphAddr = mapHostToPeripheral(p_bufferTable);

    assert((reinterpret_cast<uintptr_t>(p_bufferTable) & 0b111) == 0);
    assert((btablePeriphAddr & 0b111) == 0);

    this->m_usbCore.BTABLE = btablePeriphAddr;
}

void
Device::enableFunction(void) const {
    this->m_usbCore.DADDR |= USB_DADDR_EF;
}

void
Device::disableFunction(void) const {
    this->m_usbCore.DADDR &= ~USB_DADDR_EF;
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
