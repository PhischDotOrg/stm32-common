/*-
 * $Copyright$
-*/

#include "spi/SpiAccessViaSTM32F4.hpp"
#include "spi/SpiAccess.hpp"

#include "stm32f4xx.h"

#include <assert.h>

#define SPI_RETURNCODE(_x) (0x80000000 | ((_x) << 16) | __LINE__)

namespace spi {

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::SpiAccessViaSTM32F4(SPI_TypeDef * const p_spi, gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso)
  : m_spi(p_spi), m_sclk(p_sclk), m_nsel(p_nsel), m_mosi(p_mosi), m_miso(p_miso), m_shiftTimeoutInMs(500) {
    this->m_txBufferEmpty = xSemaphoreCreateBinary();
    this->m_rxBufferNotEmpty = xSemaphoreCreateBinary();
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::~SpiAccessViaSTM32F4() {
    vSemaphoreDelete(this->m_rxBufferNotEmpty);
    vSemaphoreDelete(this->m_txBufferEmpty);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::initialize(const BaudRatePrescaler_e p_prescaler /* = e_SpiPrescaler128 */) const {
    /*
     * I've found that my test setup can go up to a pre-scaler of 4, anything
     * faster than that doesn't work.
     * 
     * I've also found that I need to set the SPI_CR1_SSI bit here. Not sure
     * why this is, could be a bug in my own code.
     */
    this->m_spi->CR1 &= ~SPI_CR1_BR;
    this->m_spi->CR1 |= p_prescaler << 3;
    this->m_spi->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR);
    this->m_spi->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;
    this->enable();

    /* Clear RX Buffer in Hardware */
    while ((this->m_spi->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) != SPI_SR_TXE) {
        while (this->m_spi->DR != 0) ;
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::terminate(void) const {
    m_nsel.disable();
    this->disable();
}

/*******************************************************************************
 *
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
int 
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::select(const uint8_t p_device) const {
    assert(p_device == 0);
    
    this->m_spi->CR2 |= SPI_CR2_SSOE;

    return (0);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
int
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::deselect(const uint8_t p_device) const {
    assert(p_device == 0);

    this->m_spi->CR2 &= ~SPI_CR2_SSOE;

    return (0);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
int
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::shift(const uint8_t p_bits, uint8_t * const p_rx /* = NULL */, const uint8_t p_tx /* = 0xFFu */, const spi::Mode /* p_mode */ /* = SpiMode0 */) const {
    assert(p_bits <= this->SHIFT_MAX_BITS);
    assert(p_bits >= this->SHIFT_MIN_BITS);
    
    int rc = 0;
    uint16_t rx;

    this->waitForIdle();

    this->m_spi->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
    this->m_spi->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXNEIE;
    this->m_spi->DR = p_tx;

    if (!xSemaphoreTake(this->m_txBufferEmpty, this->m_shiftTimeoutInMs / portTICK_PERIOD_MS)) {
        rc = SPI_RETURNCODE(1);
        goto out;
    }

    if (!xSemaphoreTake(this->m_rxBufferNotEmpty, this->m_shiftTimeoutInMs / portTICK_PERIOD_MS)) {
        rc = SPI_RETURNCODE(2);
        goto out;
    }

    rx = this->m_spi->DR;
    if (p_rx != NULL) {
        *p_rx = rx;
    }

    this->waitForIdle();

out:
    assert(!rc);
    return (rc);
}

/*******************************************************************************
 *
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::enable(void) const {
    this->m_spi->CR1 |= SPI_CR1_SPE;
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::disable(void) const {
    this->waitForIdle();

    this->m_spi->CR1 &= ~SPI_CR1_SPE;
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::waitForIdle(void) const {
    /*
     * Wait until the RXNE and BUSY flags go off in the status register and the
     * TXE flag goes on.
     */
    while ((this->m_spi->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) != SPI_SR_TXE) ;
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
uint16_t
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::setupIrqMask(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
    uint16_t mask = 0;
    
    if (p_rxNotEmpty) {
        mask |= SPI_CR2_RXNEIE;
    }
    if (p_txEmpty) {
        mask |= SPI_CR2_TXEIE;
    }
    if (p_error) {
        mask |= SPI_CR2_ERRIE;        
    }

    return mask;
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::enableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
    this->m_spi->CR2 |= this->setupIrqMask(p_rxNotEmpty, p_txEmpty, p_error);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::disableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
    this->m_spi->CR2 &= ~this->setupIrqMask(p_rxNotEmpty, p_txEmpty, p_error);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT, typename DmaChannelT>
void
SpiAccessViaSTM32F4<useDmaT, DmaChannelT>::handleIrq(void) {
    if (this->m_spi->SR & SPI_SR_RXNE) {
        this->disableIrq(true, false, false);
        xSemaphoreGiveFromISR(this->m_rxBufferNotEmpty, NULL);
    }

    if (this->m_spi->SR & SPI_SR_TXE) {
        this->disableIrq(false, true, false);
        xSemaphoreGiveFromISR(this->m_txBufferEmpty, NULL);
    }
}

/*******************************************************************************
 * DMA-specialized Implementation
 ******************************************************************************/
template<typename DmaChannelT>
SpiAccessViaSTM32F4<true, DmaChannelT>::SpiAccessViaSTM32F4(SPI_TypeDef * const p_spi, DmaChannelT &p_txDmaChannel, DmaChannelT &p_rxDmaChannel,
  gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso)
  : SpiAccessViaSTM32F4Base(p_spi, p_sclk, p_nsel, p_mosi, p_miso),
  m_txDmaChannel(p_txDmaChannel), m_rxDmaChannel(p_rxDmaChannel), m_dummyTx(-1), m_dummyRx(0), m_spiError(0),
  m_rxDmaStatus(dma::e_DmaUndefined), m_txDmaStatus(dma::e_DmaUndefined), m_txCallback(*this, 0u), m_rxCallback(*this, 1u) {
    this->m_event = xSemaphoreCreateCounting(8, 0);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
SpiAccessViaSTM32F4<true, DmaChannelT>::~SpiAccessViaSTM32F4() {
    vSemaphoreDelete(this->m_event);
}

/*******************************************************************************
 * 
 ******************************************************************************/
extern "C" {
    extern char bstack, estack;
}

template<typename DmaChannelT>
bool
SpiAccessViaSTM32F4<true, DmaChannelT>::addressIsDmaCapable(const void * const p_addr) const {
    assert(&bstack == (void *) (intptr_t) 0x10000000);
    assert(&estack == (void *) (intptr_t) (0x10000000 + 0x10000));

    return (p_addr < &bstack) || (p_addr > &estack);
}

template<typename DmaChannelT>
int
SpiAccessViaSTM32F4<true, DmaChannelT>::shift(const uint32_t p_bits, uint8_t * const  p_rx /* = NULL */, const uint8_t * const p_tx /* = NULL */, const spi::Mode p_mode /* = SpiMode0 */) {
    m_useDma = true;

    if (m_useDma && (p_tx != NULL)) {
        m_useDma = addressIsDmaCapable(p_tx);
    }
    if (m_useDma && (p_rx != NULL)) {
        m_useDma = addressIsDmaCapable(p_rx);
    }

    if (m_useDma) {
        return this->shiftViaDma(p_bits, p_rx, p_tx, p_mode);
    } else {
        /* Do "conventional" transfer */
        int rc = 0;
        ssize_t len = ShiftHelperT<false, SpiAccessViaSTM32F4<false, void> >::shift(this, p_bits, p_tx, p_rx, p_mode);
        if (len != static_cast<ssize_t>(p_bits)) {
            rc = -1;
        }
        return rc;
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
int
SpiAccessViaSTM32F4<true, DmaChannelT>::shiftViaDma(const uint32_t p_bits, uint8_t * const  p_rx, const uint8_t * const p_tx, const spi::Mode /* p_mode */) {
    assert((p_bits % this->SHIFT_MIN_BITS) == 0);
    assert(p_bits < (1 << 16));

    int rc = -1;
    
    this->waitForIdle();

    size_t length = p_bits / 8;

    this->m_spi->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);
    
    this->m_rxDmaChannel.setup(dma::e_PeripheralToMemory, length);
    this->m_rxDmaChannel.setupSource(const_cast<const uint16_t * const>(&this->m_spi->DR), 1, false);
    this->m_rxDmaChannel.setupFifo(dma::e_Single, dma::e_Quarter);
    if (p_rx != NULL) {
        this->m_rxDmaChannel.setupTarget(p_rx, 1, true);
    } else {
        this->m_rxDmaChannel.setupTarget(&this->m_dummyRx, 1, false);        
    }

    this->m_txDmaChannel.setup(dma::e_MemoryToPeripheral, length);
    this->m_txDmaChannel.setupTarget(const_cast<uint16_t * const>(&this->m_spi->DR), 1, false);
    this->m_txDmaChannel.setupFifo(dma::e_Single, dma::e_Quarter);
    if (p_tx != NULL) {
        this->m_txDmaChannel.setupSource(p_tx, 1, true);
    } else {
        this->m_txDmaChannel.setupSource(&this->m_dummyTx, 1, false);
    }

    this->m_spiError = 0;
    this->m_txDmaStatus = dma::e_DmaUndefined;
    this->m_rxDmaStatus = dma::e_DmaUndefined;
    rc = 0;

    this->enableIrq(false, false, true);
    this->enableDma(true, true);
    this->m_rxDmaChannel.start(&this->m_rxCallback);
    this->m_txDmaChannel.start(&this->m_txCallback);
    
    do {
        if (!xSemaphoreTake(this->m_event, this->m_shiftTimeoutInMs / portTICK_PERIOD_MS)) {
            rc = SPI_RETURNCODE(1);
            assert(false);
            goto out;
        }

        if (this->m_spiError != 0) {
            rc = SPI_RETURNCODE(2);
            break;
        }

        if (this->m_txDmaStatus == dma::e_DmaError) {
            rc = SPI_RETURNCODE(3);
            break;
        }
        
        if (this->m_rxDmaStatus == dma::e_DmaError) {
            rc = SPI_RETURNCODE(4);
            break;
        }
    } while((this->m_rxDmaStatus != dma::e_DmaComplete) && (this->m_txDmaStatus != dma::e_DmaComplete));

    
    /*
     * If the SPI Shift Frequency is slow (relative to the CPU), then we may have
     * seen and handled the DMA completion interrupts before the SPI engine has
     * shifted out the last bytes to be transferred.
     * 
     * The datasheet specifically mentions to first wait for the TXE flag to
     * turn on, then for the BUSY bit to go off. See section 27.3.9.
     */
    while ((this->m_spi->SR & SPI_SR_TXE) != SPI_SR_TXE);

    this->waitForIdle();

out:
    this->m_txDmaChannel.stop();
    this->m_rxDmaChannel.stop();

    this->disableDma(true, true);
    this->disableIrq(false, false, true);

    return (rc);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
void
SpiAccessViaSTM32F4<true, DmaChannelT>::enableDma(const bool p_rx, const bool p_tx) const {
    if (p_rx) {
        this->m_spi->CR2 |= SPI_CR2_RXDMAEN;
    }

    if (p_tx) {
        this->m_spi->CR2 |= SPI_CR2_TXDMAEN;
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
void
SpiAccessViaSTM32F4<true, DmaChannelT>::disableDma(const bool p_rx, const bool p_tx) const {
    if (p_rx) {
        this->m_spi->CR2 &= ~SPI_CR2_RXDMAEN;
    }

    if (p_tx) {
        this->m_spi->CR2 &= ~SPI_CR2_TXDMAEN;
    }    
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
void
SpiAccessViaSTM32F4<true, DmaChannelT>::handleIrq(void) {
    unsigned status = this->m_spi->SR;

    if (this->m_useDma) {
        if (status & (SPI_SR_MODF | SPI_SR_OVR | SPI_SR_CRCERR)) {
            this->disableIrq(false, false, true);

            this->m_spiError = this->m_spi->SR;
            xSemaphoreGiveFromISR(this->m_event, NULL);
        }
    } else {
        SpiAccessViaSTM32F4<false, void>::handleIrq();
    }
}

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename DmaChannelT>
void
SpiAccessViaSTM32F4<true, DmaChannelT>::notify(const dma::DmaTransferStatus_t p_status, const unsigned p_id) {
    switch (p_id) {
    case 0:
        this->m_txDmaStatus = p_status;
        break;
    case 1:
        this->m_rxDmaStatus = p_status;
        break;
    default:
        /* Ignore */
        break;
    }

    BaseType_t higherPrioTaskWoken;
    if (xSemaphoreGiveFromISR(this->m_event, &higherPrioTaskWoken) == pdTRUE) {
        portYIELD_FROM_ISR(higherPrioTaskWoken);
    }
}

} /* namespace spi */
