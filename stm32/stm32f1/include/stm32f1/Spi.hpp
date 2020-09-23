/*-
 * $Copyright$
-*/

#ifndef _STM32_SPI_ENGINE_HPP_9C44906F_07F2_4092_97FC_94EC888B5879
#define _STM32_SPI_ENGINE_HPP_9C44906F_07F2_4092_97FC_94EC888B5879

#define SPI_RETURNCODE(_x) (0x80000000 | ((_x) << 16) | __LINE__)

#include <stm32/Engine.hpp>

#include <spi/Spi.hpp>

#include <stm32f4xx.h>

#include <stm32/dma/Types.hpp>

/*****************************************************************************/
namespace stm32 {
    namespace f1 {
/*****************************************************************************/

/*****************************************************************************/
class SpiEngine {
protected:
    SPI_TypeDef &m_spi;

    enum class Irq_e : uint16_t {
        e_RxNotEmpty    = SPI_CR2_RXNEIE,
        e_TxEmpty       = SPI_CR2_TXEIE,
        e_Error         = SPI_CR2_ERRIE,
    };

    void
    enableIrq(Irq_e p_irq) const {
        m_spi.CR2 |= static_cast<uint16_t>(p_irq);
    }

    void
    disableIrq(Irq_e p_irq) const {
        m_spi.CR2 &= ~static_cast<uint16_t>(p_irq);
    }

    void
    pollForIdle(void) const {
        /*
        * Wait until the RXNE and BUSY flags go off in the status register and the
        * TXE flag goes on.
        */
        while ((m_spi.SR & (SPI_SR_BSY | SPI_SR_TXE | SPI_SR_RXNE)) != SPI_SR_TXE) m_spi.DR;
    }

    void
    setupSpiMode(const ::spi::Mode p_mode) const {
        m_spi.CR1 &= ~(SPI_CR1_CPHA | SPI_CR1_CPOL);

        switch(p_mode) {
        case ::spi::Mode::SpiMode0:  /* CPOL = 0, CPHA = 0 */
            /* Nothing to do here. */
            break;
        case ::spi::Mode::SpiMode1:  /* CPOL = 0, CPHA = 1 */
            m_spi.CR1 |= SPI_CR1_CPHA;
            break;
        case ::spi::Mode::SpiMode2:  /* CPOL = 1, CPHA = 0 */
            m_spi.CR1 |= SPI_CR1_CPOL;
            break;
        case ::spi::Mode::SpiMode3:  /* CPOL = 1, CPHA = 1 */
            m_spi.CR1 |= (SPI_CR1_CPOL | SPI_CR1_CPHA);
            break;
        }
    }

    void
    enableRxDma(void) const {
        m_spi.CR2 |= SPI_CR2_RXDMAEN;
        enableIrq(Irq_e::e_RxNotEmpty);
    }

    void
    disableRxDma(void) const {
        disableIrq(Irq_e::e_RxNotEmpty);
        m_spi.CR2 &= ~SPI_CR2_RXDMAEN;
    }

    void
    enableTxDma(void) const {
        m_spi.CR2 |= SPI_CR2_TXDMAEN;
        enableIrq(Irq_e::e_TxEmpty);
    }

    void
    disableTxDma(void) const {
        disableIrq(Irq_e::e_TxEmpty);
        m_spi.CR2 &= ~SPI_CR2_TXDMAEN;
    }

    void
    setupMaster(void) const {
        m_spi.CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_MSTR);
        m_spi.I2SCFGR &= ~SPI_I2SCFGR_I2SMOD;

        enable();
        pollForIdle();
    }

    void
    disable(void) const {
        pollForIdle();

        m_spi.CR1 &= ~SPI_CR1_SPE;
    }

    void
    enable(void) const {
        m_spi.CR1 |= SPI_CR1_SPE;
    }

public:
    enum class BaudRatePrescaler_e : uint8_t {
        e_SpiPrescaler2     = 0b000,
        e_SpiPrescaler4     = 0b001,
        e_SpiPrescaler8     = 0b010,
        e_SpiPrescaler16    = 0b011,
        e_SpiPrescaler32    = 0b100,
        e_SpiPrescaler64    = 0b101,
        e_SpiPrescaler128   = 0b110,
        e_SpiPrescaler256   = 0b111,
    };

    SpiEngine(SPI_TypeDef *p_spi) : m_spi(*p_spi) {
    }

    void
    setPrescaler(SpiEngine::BaudRatePrescaler_e p_prescaler) const {
        m_spi.CR1 &= ~SPI_CR1_BR_Msk;
        m_spi.CR1 |= static_cast<unsigned>(p_prescaler) << SPI_CR1_BR_Pos;
    }

    int
    select(const uint8_t /* p_device */) const {
        m_spi.CR2 |= SPI_CR2_SSOE;

        return (0);
    }

    int
    deselect(const uint8_t /* p_device */) const {
        m_spi.CR2 &= ~SPI_CR2_SSOE;

        return (0);
    }

    void
    handleIrq(void) {
        assert(false);
    }
};
/*****************************************************************************/

/*****************************************************************************/
template<
    intptr_t Address,
    typename RccT,
    typename PinT
>
class SpiT : public EngineT<Address>, public SpiEngine {
    const RccT &    m_rcc;
    const PinT &    m_sclk;
    const PinT &    m_nsel;
    const PinT &    m_mosi;
    const PinT &    m_miso;

public:
    static const unsigned SHIFT_MIN_BITS = 8;
    static const unsigned SHIFT_MAX_BITS = 0;

    SpiT(RccT &p_rcc, const PinT &p_sclk, const PinT &p_nsel, const PinT &p_mosi, const PinT &p_miso,
      const SpiEngine::BaudRatePrescaler_e p_prescaler)
        : SpiEngine(reinterpret_cast<SPI_TypeDef *>(Address)),
          m_rcc(p_rcc), m_sclk(p_sclk), m_nsel(p_nsel), m_mosi(p_mosi), m_miso(p_miso) {
            m_rcc.enableEngine(* static_cast<EngineT<Address> *>(this));

            setPrescaler(p_prescaler);
            setupMaster();
        

            m_sclk.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            m_nsel.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            m_mosi.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
            m_miso.selectAlternateFn(static_cast<const EngineT<Address> &>(*this));
    };

    ~SpiT() {
        m_sclk.disable();
        m_nsel.disable();
        m_mosi.disable();
        m_miso.disable();

        m_rcc.disableEngine(* static_cast<EngineT<Address> *>(this));
    };

    int
    shift(const uint8_t p_bits, uint8_t * const p_rx = nullptr, const uint8_t *p_tx = nullptr, const ::spi::Mode p_mode = ::spi::SpiMode0) const {
        int rc = -1;

        assert((p_bits > 0) && (p_bits < (1 << 16)));
        assert((p_bits % 8) == 0);
        assert((p_tx != nullptr) || (p_rx != nullptr));

        pollForIdle();
        setupSpiMode(p_mode);

        size_t length = p_bits / 8;
        assert(length > 0);

        enableIrq(Irq_e::e_Error);

        for (unsigned idx = 0; idx < length; idx++) {
            /* Wait for room in the Tx FIFO */
            while (!(m_spi.SR & SPI_SR_TXE)) ;

            if (p_tx != nullptr) {
                m_spi.DR = p_tx[idx];
            }

            if (p_rx != nullptr) {
                /* Wait until data is available */
                while (!(m_spi.SR & SPI_SR_RXNE)) ;

                p_rx[idx] = m_spi.DR;
            }
        }

        pollForIdle();

        disableIrq(Irq_e::e_Error);

        return (rc);
    }
};
/*****************************************************************************/

/*****************************************************************************/
template<
    intptr_t Address,
    typename RccT,
    typename DmaTxChannelT,
    typename DmaRxChannelT,
    typename PinT
>
class SpiViaDmaT : public SpiT<Address, RccT, PinT> {
    const DmaTxChannelT &   m_txDmaChannel;
    const DmaRxChannelT &   m_rxDmaChannel;

public:
    SpiViaDmaT(RccT &p_rcc, DmaTxChannelT &p_txDmaChannel, DmaRxChannelT &p_rxDmaChannel,
      const PinT &p_sclk, const PinT &p_nsel, const PinT &p_mosi, const PinT &p_miso,
      const SpiEngine::BaudRatePrescaler_e p_prescaler)
        : SpiT<Address, RccT, PinT>(p_rcc, p_sclk, p_nsel, p_mosi, p_miso, p_prescaler),
          m_txDmaChannel(p_txDmaChannel), m_rxDmaChannel(p_rxDmaChannel) {
    };

    int
    shift(const uint8_t p_bits, uint8_t * const p_rx = nullptr, const uint8_t *p_tx = nullptr, const ::spi::Mode p_mode = ::spi::SpiMode0) const {
        int rc = -1;

        assert((p_bits > 0) && (p_bits < (1 << 16)));
        assert((p_bits % 8) == 0);
        assert((p_tx != nullptr) || (p_rx != nullptr));

        this->pollForIdle();
        this->setupSpiMode(p_mode);

        size_t length = p_bits / 8;

        this->enableIrq(SpiEngine::Irq_e::e_Error);

        if (p_rx != nullptr) {
            this->enableRxDma();

            m_rxDmaChannel.setup(dma::e_PeripheralToMemory, length);
            m_rxDmaChannel.setupSource(const_cast<const uint32_t * const>(&this->m_spi.DR), 1, false);
            m_rxDmaChannel.setupFifo(dma::e_Single, dma::e_Quarter);
            m_rxDmaChannel.setupTarget(p_rx, 1, true);

            // m_rxDmaChannel.start(&this->m_rxCallback);
        } else {
            this->disableRxDma();
        }

        if (p_tx != nullptr) {
            this->enableTxDma();
            this->enableIrq(SpiEngine::Irq_e::e_TxEmpty);

            m_txDmaChannel.setup(dma::e_MemoryToPeripheral, length);
            m_txDmaChannel.setupTarget(const_cast<uint32_t * const>(&this->m_spi.DR), 1, false);
            m_txDmaChannel.setupFifo(dma::e_Single, dma::e_Quarter);
            m_txDmaChannel.setupSource(p_tx, 1, true);

            // m_txDmaChannel.start(&this->m_txCallback);
        } else {
            this->disableTxDma();
        }

        /* TODO */
        // if (p_rx != nullptr) {
        //     waitForRxComplete();
        // }

        /* TODO */
        // if (p_tx != nullptr) {
        //     waitForTxComplete();
        // }

        /* TODO Check for Error */

        /*
        * If the SPI Shift Frequency is slow (relative to the CPU), then we may have
        * seen and handled the DMA completion interrupts before the SPI engine has
        * shifted out the last bytes to be transferred.
        *
        * The datasheet specifically mentions to first wait for the TXE flag to
        * turn on, then for the BUSY bit to go off. See section 27.3.9.
        */
        while ((this->m_spi.SR & SPI_SR_TXE) != SPI_SR_TXE);

        this->pollForIdle();

    // out:
        m_txDmaChannel.stop();
        m_rxDmaChannel.stop();

        this->disableRxDma();
        this->disableTxDma();

        this->disableIrq(SpiEngine::Irq_e::e_Error);

        return (rc);
    }
};
/*****************************************************************************/

/*****************************************************************************/
    } /* namespace f1 */
} /* namespace stm32 */
/*****************************************************************************/
#endif /* _STM32_SPI_ENGINE_HPP_9C44906F_07F2_4092_97FC_94EC888B5879 */
