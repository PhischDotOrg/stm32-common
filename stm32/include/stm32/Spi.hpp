/*-
 * $Copyright$
-*/

#ifndef _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b
#define _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b

#define SPI_RETURNCODE(_x) (0x80000000 | ((_x) << 16) | __LINE__)

extern "C" {
    extern char bstack, estack;
}

#include <spi/Spi.hpp>

#include <stm32/dma/Types.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

// #if !defined(HOSTBUILD)
#include <FreeRTOS.h>
#include <semphr.h>
// #else
// typedef void * SemaphoreHandle_t;
// #endif /* defined(HOSTBUILD) */

#include "stm32f4xx.h"

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>
#include <stddef.h>

/*****************************************************************************/
namespace stm32 {
    namespace spi {
/*****************************************************************************/

/*******************************************************************************
 *
 ******************************************************************************/
template<bool useDmaT = false, typename DmaTxChannelT = void, typename DmaRxChannelT = void>
class SpiAccessViaSTM32F4 {
public:
    enum BaudRatePrescaler_e {
        e_SpiPrescaler32    = 0x0,
        e_SpiPrescaler64    = 0x1,
        e_SpiPrescaler128   = 0x2,
        e_SpiPrescaler256   = 0x3,
        e_SpiPrescaler512   = 0x4,
        e_SpiPrescaler1024  = 0x5,
        e_SpiPrescaler2048  = 0x6,
        e_SpiPrescaler4096  = 0x7
    };

    void handleIrq(void) {
        if (this->m_spi->SR & SPI_SR_RXNE) {
            this->disableIrq(true, false, false);
            xSemaphoreGiveFromISR(this->m_rxBufferNotEmpty, NULL);
        }

        if (this->m_spi->SR & SPI_SR_TXE) {
            this->disableIrq(false, true, false);
            xSemaphoreGiveFromISR(this->m_txBufferEmpty, NULL);
        }
    }

    int select(const uint8_t /* p_device */) const {
        this->m_spi->CR2 |= SPI_CR2_SSOE;

        return (0);
    }

    int deselect(const uint8_t /* p_device */) const {
        this->m_spi->CR2 &= ~SPI_CR2_SSOE;

        return (0);
    }

    int shift(const uint8_t /* p_bits */, uint8_t * const p_rx = NULL, const uint8_t p_tx = 0xFFu, const ::spi::Mode /* p_mode */ = ::spi::SpiMode0) const {
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

    uint32_t getShiftTimeout(void) const { return this->m_shiftTimeoutInMs; };
    void     setShiftTimeout(const uint32_t p_shiftTimeoutInMs) { this->m_shiftTimeoutInMs = p_shiftTimeoutInMs; }

    static const unsigned SHIFT_MIN_BITS = 8;
    static const unsigned SHIFT_MAX_BITS = 8;

protected:
    SpiAccessViaSTM32F4(SPI_TypeDef * const p_engine) : m_spi(p_engine), m_shiftTimeoutInMs(500) {
        this->m_txBufferEmpty = xSemaphoreCreateBinary();
        this->m_rxBufferNotEmpty = xSemaphoreCreateBinary();
    }

    ~SpiAccessViaSTM32F4() {
        vSemaphoreDelete(this->m_rxBufferNotEmpty);
        vSemaphoreDelete(this->m_txBufferEmpty);
    }

    SPI_TypeDef * const m_spi;

    const uint32_t      m_shiftTimeoutInMs;

    SemaphoreHandle_t   m_txBufferEmpty;
    SemaphoreHandle_t   m_rxBufferNotEmpty;

    void initialize(const BaudRatePrescaler_e p_prescaler = e_SpiPrescaler128) const {
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

    void terminate(void) const {
        /* FIXME */
        // m_nsel.disable();
        this->disable();
    }

    void waitForIdle(void) const {
        /*
        * Wait until the RXNE and BUSY flags go off in the status register and the
        * TXE flag goes on.
        */
        while ((this->m_spi->SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) != SPI_SR_TXE) ;
    }

    void disable(void) const {
        this->waitForIdle();

        this->m_spi->CR1 &= ~SPI_CR1_SPE;
    }

    void enable(void) const {
        this->m_spi->CR1 |= SPI_CR1_SPE;
    }

    uint16_t setupIrqMask(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
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

    void enableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
        this->m_spi->CR2 |= this->setupIrqMask(p_rxNotEmpty, p_txEmpty, p_error);
    }

    void disableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const {
        this->m_spi->CR2 &= ~this->setupIrqMask(p_rxNotEmpty, p_txEmpty, p_error);
    }
};

typedef class SpiAccessViaSTM32F4<false, void> SpiAccessViaSTM32F4Base;

/*******************************************************************************
 * DMA-enabled implementation
 ******************************************************************************/
template<
    typename DmaTxChannelT,
    typename DmaRxChannelT
>
class SpiAccessViaSTM32F4<true, DmaTxChannelT, DmaRxChannelT> : public SpiAccessViaSTM32F4Base {
public:
    int
    shift(const uint32_t p_bits, uint8_t * const p_rx = NULL, const uint8_t * const p_tx = NULL, const ::spi::Mode p_mode = ::spi::SpiMode0) {
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
            ssize_t len = ::spi::ShiftHelperT<false, SpiAccessViaSTM32F4<false, void> >::shift(this, p_bits, p_tx, p_rx, p_mode);
            if (len != static_cast<ssize_t>(p_bits)) {
                rc = -1;
            }
            return rc;
        }
    }

    void handleIrq(void) {
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

    static const unsigned SHIFT_MIN_BITS = 8;
    static const unsigned SHIFT_MAX_BITS = 0;

protected:
    SpiAccessViaSTM32F4(SPI_TypeDef * const p_engine, DmaTxChannelT &p_txDmaChannel, DmaRxChannelT &p_rxDmaChannel)
        : SpiAccessViaSTM32F4Base(p_engine), m_txDmaChannel(p_txDmaChannel), m_rxDmaChannel(p_rxDmaChannel), m_dummyTx(-1), m_dummyRx(0), m_spiError(0),
          m_rxDmaStatus(dma::e_DmaUndefined), m_txDmaStatus(dma::e_DmaUndefined), m_txCallback(*this, 0u), m_rxCallback(*this, 1u) {
        this->m_event = xSemaphoreCreateCounting(8, 0);
    }

    ~SpiAccessViaSTM32F4() {
        vSemaphoreDelete(this->m_event);
    }

private:
    class ChannelRxCallback : public DmaRxChannelT::Callback {
    private:
        SpiAccessViaSTM32F4 &m_obj;
        const unsigned m_id;
    public:
        ChannelRxCallback(SpiAccessViaSTM32F4 &p_obj, const unsigned p_id) : m_obj(p_obj), m_id(p_id) {};
        virtual ~ChannelRxCallback() {};

        virtual void notify(const dma::DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status, this->m_id);
        }
    };

    class ChannelTxCallback : public DmaTxChannelT::Callback {
    private:
        SpiAccessViaSTM32F4 &m_obj;
        const unsigned m_id;
    public:
        ChannelTxCallback(SpiAccessViaSTM32F4 &p_obj, const unsigned p_id) : m_obj(p_obj), m_id(p_id) {};
        virtual ~ChannelTxCallback() {};

        virtual void notify(const dma::DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status, this->m_id);
        }
    };

    DmaTxChannelT &     m_txDmaChannel;
    DmaRxChannelT &     m_rxDmaChannel;

    const uint8_t       m_dummyTx;
    uint8_t             m_dummyRx;

    uint16_t                    m_spiError;
    dma::DmaTransferStatus_t    m_rxDmaStatus;
    dma::DmaTransferStatus_t    m_txDmaStatus;

    SemaphoreHandle_t   m_event;

    ChannelTxCallback   m_txCallback;
    ChannelRxCallback   m_rxCallback;

    bool                m_useDma;

    void enableDma(const bool p_rx, const bool p_tx) const {
        if (p_rx) {
            this->m_spi->CR2 |= SPI_CR2_RXDMAEN;
        }

        if (p_tx) {
            this->m_spi->CR2 |= SPI_CR2_TXDMAEN;
        }
    }

    void disableDma(const bool p_rx, const bool p_tx) const {
        if (p_rx) {
            this->m_spi->CR2 &= ~SPI_CR2_RXDMAEN;
        }

        if (p_tx) {
            this->m_spi->CR2 &= ~SPI_CR2_TXDMAEN;
        }
    }

    void notify(const dma::DmaTransferStatus_t p_status, const unsigned p_id) {
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

    int shiftViaDma(const uint32_t p_bits, uint8_t * const p_rx, const uint8_t * const p_tx, const ::spi::Mode /* p_mode */) {
        assert((p_bits % this->SHIFT_MIN_BITS) == 0);
        assert(p_bits < (1 << 16));

        int rc = -1;

        this->waitForIdle();

        size_t length = p_bits / 8;

        this->m_spi->CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);

        this->m_rxDmaChannel.setup(dma::e_PeripheralToMemory, length);
        this->m_rxDmaChannel.setupSource(const_cast<const uint32_t * const>(&this->m_spi->DR), 1, false);
        this->m_rxDmaChannel.setupFifo(dma::e_Single, dma::e_Quarter);
        if (p_rx != NULL) {
            this->m_rxDmaChannel.setupTarget(p_rx, 1, true);
        } else {
            this->m_rxDmaChannel.setupTarget(&this->m_dummyRx, 1, false);
        }

        this->m_txDmaChannel.setup(dma::e_MemoryToPeripheral, length);
        this->m_txDmaChannel.setupTarget(const_cast<uint32_t * const>(&this->m_spi->DR), 1, false);
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

    bool addressIsDmaCapable(const void * const /* p_addr */) const {
        // assert(&bstack == (void *) (intptr_t) 0x10000000);
        // assert(&estack == (void *) (intptr_t) (0x10000000 + 0x10000));

        // return (p_addr < &bstack) || (p_addr > &estack);
        return true;
    }
};

/*
 * Required for compilation when DmaChannelT resolves to "void". Mostly for unit
 * tests.
 */
template<> class SpiAccessViaSTM32F4<true, void> : public SpiAccessViaSTM32F4Base { };

// typedef SpiAccessViaSTM32F4<true, dma::DmaChannel> SpiAccessViaSTM32F4Dma;

/*******************************************************************************
 *
 ******************************************************************************/
template<
    intptr_t Address,
    typename RccT /* = devices::RccViaSTM32F4 */,
    typename DmaTxChannelT,
    typename DmaRxChannelT,
    typename PinT /* gpio::GpioPin */
>
class SpiAccessViaSTM32F4Fixed : public EngineT<Address>, public SpiAccessViaSTM32F4<true, DmaTxChannelT, DmaRxChannelT> {
private:
    const RccT &    m_rcc;
    // PinT &          m_sclk;
    // PinT &          m_nsel;
    // PinT &          m_mosi;
    // PinT &          m_miso;

public:
    SpiAccessViaSTM32F4Fixed(RccT &p_rcc, DmaTxChannelT &p_txDmaChannel, DmaRxChannelT &p_rxDmaChannel,
      const PinT &p_sclk, const PinT &p_nsel, const PinT &p_mosi, const PinT &p_miso,
      const typename SpiAccessViaSTM32F4<true, DmaTxChannelT, DmaRxChannelT>::BaudRatePrescaler_e p_prescaler = SpiAccessViaSTM32F4<true, DmaTxChannelT, DmaRxChannelT>::e_SpiPrescaler128)
        : SpiAccessViaSTM32F4<true, DmaTxChannelT, DmaRxChannelT>(reinterpret_cast<SPI_TypeDef *>(Address), p_txDmaChannel, p_rxDmaChannel), m_rcc(p_rcc) {
            m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

            p_sclk.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            /*
            * FIXME Is this still guaranteed?
            * During debug I've found that at least SPI2 needs a Pull-up on the NSEL line; otherwise the chip
            * select will always stay low.
            */
            p_nsel.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            p_mosi.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            p_miso.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));

            this->initialize(p_prescaler);
    };

    ~SpiAccessViaSTM32F4Fixed() {
        this->terminate();

        /* FIXME Disable Alternate Function on I/O Pins */
        // m_sclk.disable();
        // m_nsel.disable();
        // m_mosi.disable();
        // m_miso.disable();

        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    };
};

/*****************************************************************************/
    } /* namespace spi */
} /* namespace stm32 */
/*****************************************************************************/

#endif /* _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b */
