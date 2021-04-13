/*-
 * $Copyright$
-*/

#include "usb/Endpoint.hpp"
#include "usb/Device.hpp"

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
        namespace usb {
/*****************************************************************************/

void
Endpoint::reset(void) const {
    disable();  
    clrCtrRx();
    clrCtrTx();

    setAddress(this->m_endpointNumber);
}

void
Endpoint::rxEnable(void) const {
    setTxStatus(TxStatus_t::e_Nak);
    this->setRxStatus(RxStatus_t::e_Valid);
}

void
Endpoint::txEnable(void) const {
    setRxStatus(RxStatus_t::e_Nak);
    this->setTxStatus(TxStatus_t::e_Valid);
}

void
Endpoint::disable(void) const {
    setRxStatus(RxStatus_t::e_Disabled);
    setTxStatus(TxStatus_t::e_Disabled);
}

void
Endpoint::setEndpointType(EndpointType_t p_endpointType) const {
    this->setEPnR(USB_EP_T_FIELD_Msk, static_cast<uint16_t>(p_endpointType) << USB_EP_T_FIELD_Pos);
}

void
Endpoint::setRxStatus(RxStatus_t p_rxStatus) const {
    this->setEPnR(USB_EPRX_STAT_Msk, static_cast<uint16_t>(p_rxStatus) << USB_EPRX_STAT_Pos);
}

void
Endpoint::setTxStatus(TxStatus_t p_txStatus) const {
    this->setEPnR(USB_EPTX_STAT_Msk, static_cast<uint16_t>(p_txStatus) << USB_EPTX_STAT_Pos);
}

void
Endpoint::clrCtrRx(void) const {
    this->setEPnR(USB_EP_CTR_RX_Msk, 0);
}

void
Endpoint::clrCtrTx(void) const {
    this->setEPnR(USB_EP_CTR_TX_Msk, 0);
}

void
Endpoint::setAddress(const unsigned p_endpointNumber) const {
    this->setEPnR(USB_EPADDR_FIELD_Msk, p_endpointNumber);
}

void
Endpoint::clearInterrupt(Interrupt_t p_irq) const {
    uint16_t irq = static_cast<uint16_t>(p_irq);
    if (p_irq == Interrupt_e::e_SetupComplete) {
        irq = static_cast<uint16_t>(Interrupt_e::e_CorrectTransferRx);
    }
    this->setEPnR(irq, 0);
}

/*****************************************************************************/
// The below methods are copied and modified from the f1usb Repository.
//
// License Text from the f1usb Repository follows.
/*****************************************************************************/

/*
 * Copyright (c) 2017, Niklas Gürtler
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Da die einzelnen Bits der EPnR-Register auf verschiedene Arten zu setzen sind und beim Schreiben
 * genau darauf geachtet werden muss dass nicht versehentlich die falschen Bits geschrieben werden,
 * kapselt diese Funktion den Schreibzugriff. Der Parameter EP gibt den Index des Registers an. Alle
 * Bits, die geschrieben werden sollen (unabhängig vom Wert) müssen in "mask" auf 1 gesetzt werden;
 * ist mask=0, wird gar nichts geschrieben. Die tatsächlich zu schreibenden Werte werden in "data"
 * angegeben. Bits welche in "mask" 0 sind, werden in "data" ignoriert. In "old" wird der vorherige
 * Zustand des Registers übergeben, falls er zuvor ohnehin schon abgefragt wurde. Ist das nicht
 * der Fall, kann die überladene Funktion ohne diesen Parameter genutzt werden.
 */
void
Endpoint::setEPnR(uint16_t mask, uint16_t data, uint16_t old) const {
    // Diese Bits werden beim Schreiben von 0 gelöscht und bleiben bei 1 unverändert.
    constexpr uint16_t rc_w0 = USB_EP_CTR_RX_Msk | USB_EP_CTR_TX_Msk;
    // Diese Bits werden beim Schreiben von 1 umgeschaltet, und bleiben bei 0 unverändert.
    constexpr uint16_t toggle = USB_EP_DTOG_RX_Msk | USB_EPRX_STAT_Msk | USB_EP_DTOG_TX_Msk | USB_EPTX_STAT_Msk;
    // Diese Bits verhalten sich "normal", d.h. der geschriebene Wert wird direkt übernommen.
    constexpr uint16_t rw = USB_EP_T_FIELD_Msk | USB_EP_KIND_Msk | USB_EPADDR_FIELD;

    // Prüfe zu löschende Bits
    uint16_t wr0 = static_cast<uint16_t> (rc_w0 & (~mask | data));
    // Bei Bits mit Umschalte-Verhalten muss der alte Zustand beachtet und per XOR verarbeitet werden
    uint16_t wr1 = (mask & toggle) & (old ^ data);
    // Bei "normalen" Bits wird der alte Zustand beibehalten oder auf Wunsch überschrieben.
    uint16_t wr2 = rw & ((old & ~mask) | data);

    // Kombiniere alle drei Schreibmethoden.
    *(this->m_register) = static_cast<uint16_t> (wr0 | wr1 | wr2);
}

/**
 * Diese Überladung kann genutzt werden, wenn der aktuelle Zustand von EPnR noch nicht abgefragt wurde;
 * in diesem Fall fragt die Funktion den Zustand ab und übergibt ihn an den "old"-Parameter.
 */
void
Endpoint::setEPnR(uint16_t mask, uint16_t data) const {
    setEPnR(mask, data, *(this->m_register));
}

/*****************************************************************************/
        } /* namespace usb */
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
