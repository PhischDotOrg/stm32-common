/*-
 * $Copyright$
-*/

#ifndef _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b
#define _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b

#include <spi/Spi.hpp>
#include <dma/DmaChannel.hpp>
#include <dma/DmaTypesViaSTM32F4.hpp>
#include <gpio/GpioPin.hpp>
#include <stm32f4/RccViaSTM32.hpp>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#if !defined(HOSTBUILD)
#include <FreeRTOS.h>
#include <semphr.h>
#else
typedef void * SemaphoreHandle_t;
#endif /* defined(HOSTBUILD) */

#include "stm32f4xx.h"

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stdint.h>
#include <stddef.h>

namespace spi {

/*******************************************************************************
 * 
 ******************************************************************************/
template<bool useDmaT = false, typename DmaChannelT = void>
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

    void handleIrq(void);

    int select(const uint8_t p_device) const;
    int deselect(const uint8_t p_device) const;

    int shift(const uint8_t p_bits, uint8_t * const p_rx = NULL, const uint8_t p_tx = 0xFFu, const spi::Mode p_mode = SpiMode0) const;

    uint32_t getShiftTimeout(void) const { return this->m_shiftTimeoutInMs; };
    void     setShiftTimeout(const uint32_t p_shiftTimeoutInMs) { this->m_shiftTimeoutInMs = p_shiftTimeoutInMs; }

    static const unsigned SHIFT_MIN_BITS = 8;
    static const unsigned SHIFT_MAX_BITS = 8;

protected:
    SpiAccessViaSTM32F4(SPI_TypeDef * const p_engine, gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso);
    ~SpiAccessViaSTM32F4();

    SPI_TypeDef * const m_spi;
    gpio::GpioPin & m_sclk;
    gpio::GpioPin & m_nsel;
    gpio::GpioPin & m_mosi;
    gpio::GpioPin & m_miso;

    uint32_t            m_shiftTimeoutInMs;

    SemaphoreHandle_t   m_txBufferEmpty;
    SemaphoreHandle_t   m_rxBufferNotEmpty;

    void initialize(const BaudRatePrescaler_e p_prescaler = e_SpiPrescaler128) const;
    void terminate(void) const;

    void waitForIdle(void) const;
    void disable(void) const;
    void enable(void) const;

    uint16_t setupIrqMask(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const;
    void enableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const;
    void disableIrq(const bool p_rxNotEmpty, const bool p_txEmpty, const bool p_error) const;
};

typedef class SpiAccessViaSTM32F4<false, void> SpiAccessViaSTM32F4Base;

/*******************************************************************************
 * DMA-enabled implementation
 ******************************************************************************/
template<typename DmaChannelT>
class SpiAccessViaSTM32F4<true, DmaChannelT> : public SpiAccessViaSTM32F4Base {
public:
    int shift(const uint32_t p_bits, uint8_t * const p_rx = NULL, const uint8_t * const p_tx = NULL, const spi::Mode p_mode = SpiMode0);

    void handleIrq(void);

    static const unsigned SHIFT_MIN_BITS = 8;
    static const unsigned SHIFT_MAX_BITS = 0;

protected:
    SpiAccessViaSTM32F4(SPI_TypeDef * const p_engine, DmaChannelT &p_txDmaChannel, DmaChannelT &p_rxDmaChannel, gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso);
    ~SpiAccessViaSTM32F4();
    
private:
    class ChannelCallback : public DmaChannelT::Callback {
    private:
        SpiAccessViaSTM32F4 &m_obj;
        const unsigned m_id;
    public:
        ChannelCallback(SpiAccessViaSTM32F4 &p_obj, const unsigned p_id) : m_obj(p_obj), m_id(p_id) {};
        virtual ~ChannelCallback() {};

        virtual void notify(const dma::DmaTransferStatus_t p_status) const {
            m_obj.notify(p_status, this->m_id);
        }
    };

    DmaChannelT &       m_txDmaChannel;
    DmaChannelT &       m_rxDmaChannel;

    const uint8_t       m_dummyTx;
    uint8_t             m_dummyRx;

    uint16_t                    m_spiError;
    dma::DmaTransferStatus_t    m_rxDmaStatus;
    dma::DmaTransferStatus_t    m_txDmaStatus;

    SemaphoreHandle_t   m_event;

    ChannelCallback     m_txCallback;
    ChannelCallback     m_rxCallback;

    bool                m_useDma;

    void enableDma(const bool p_rx, const bool p_tx) const;
    void disableDma(const bool p_rx, const bool p_tx) const;
    
    void notify(const dma::DmaTransferStatus_t p_status, const unsigned p_id);

    int shiftViaDma(const uint32_t p_bits, uint8_t * const p_rx, const uint8_t * const p_tx, const spi::Mode p_mode);

    bool addressIsDmaCapable(const void * const p_addr) const;
};

/*
 * Required for compilation when DmaChannelT resolves to "void". Mostly for unit
 * tests.
 */
template<> class SpiAccessViaSTM32F4<true, void> : public SpiAccessViaSTM32F4Base { };

typedef SpiAccessViaSTM32F4<true, dma::DmaChannel> SpiAccessViaSTM32F4Dma;

/*******************************************************************************
 *
 ******************************************************************************/
template<gpio::GpioAccessViaSTM32F4::Function_e GpioFunctionT>
class SpiAccessViaSTM32F4GpioHelperT {
public:
    static void initalize(gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso) {
        p_sclk.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
        /*
         * During debug I've found that at least SPI2 needs a Pull-up on the NSEL line; otherwise the chip
         * select will always stay low.
         */
        p_nsel.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_PullUp, GpioFunctionT);
        p_mosi.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
        p_miso.enable(gpio::GpioAccessViaSTM32F4::e_Alternate, gpio::GpioAccessViaSTM32F4::e_None, GpioFunctionT);
    }
    
    static void terminate(gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso) {
        p_sclk.disable();
        p_nsel.disable();
        p_mosi.disable();
        p_miso.disable();
    }
};

/*******************************************************************************
 * Forward declare and typedef the instance specific types for the SPI cores
 ******************************************************************************/
template<intptr_t SpiT, typename DmaChannelT = dma::DmaChannel, typename RccT = devices::RccViaSTM32F4> class SpiAccessViaSTM32F4Fixed;

#if defined(SPI1_BASE)
typedef SpiAccessViaSTM32F4Fixed<SPI1_BASE> SpiAccessViaSTM32F4_Spi1;
#endif
#if defined(SPI2_BASE)
typedef SpiAccessViaSTM32F4Fixed<SPI2_BASE> SpiAccessViaSTM32F4_Spi2;
#endif
#if defined(SPI3_BASE)
typedef SpiAccessViaSTM32F4Fixed<SPI3_BASE> SpiAccessViaSTM32F4_Spi3;
#endif

/*******************************************************************************
 * Helper Template to resolve the SPI Address to the corresponding GPIO
 * Alternate Function Defintion
 ******************************************************************************/
template<intptr_t> struct SpiAccessViaSTM32F4GpioFunction;

#if defined(SPI1_BASE)
template<> struct SpiAccessViaSTM32F4GpioFunction<SPI1_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_Spi1;
};
#endif
#if defined(SPI2_BASE)
template<> struct SpiAccessViaSTM32F4GpioFunction<SPI2_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_Spi2;
};
#endif
#if defined(SPI3_BASE)
template<> struct SpiAccessViaSTM32F4GpioFunction<SPI3_BASE> {
    static const gpio::GpioAccessViaSTM32F4::Function_e m_type = gpio::GpioAccessViaSTM32F4::e_Spi3;
};
#endif

/*******************************************************************************
 * Helper Template to resolve the SPI Address to the corresponding GPIO
 * RCC Function Defintion
 ******************************************************************************/
template<intptr_t> struct SpiAccessViaSTM32F4RccFunction;

#if defined(SPI1_BASE)
template<> struct SpiAccessViaSTM32F4RccFunction<SPI1_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Spi1;
};
#endif
#if defined(SPI2_BASE)
template<> struct SpiAccessViaSTM32F4RccFunction<SPI2_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Spi2;
};
#endif
#if defined(SPI3_BASE)
template<> struct SpiAccessViaSTM32F4RccFunction<SPI3_BASE> {
    static const auto m_type = devices::RccViaSTM32F4::Stm32FxxCpu_t::e_Spi3;
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
template<intptr_t SpiT, typename DmaChannelT /* = dma::DmaChannel */, typename RccT /* = devices::RccViaSTM32F4 */>
class SpiAccessViaSTM32F4Fixed : public SpiAccessViaSTM32F4Dma {
private:
    RccT &m_rcc;
    
public:
    SpiAccessViaSTM32F4Fixed(RccT &p_rcc, DmaChannelT &p_txDmaChannel, DmaChannelT &p_rxDmaChannel,
      gpio::GpioPin &p_sclk, gpio::GpioPin &p_nsel, gpio::GpioPin &p_mosi, gpio::GpioPin &p_miso,
      const typename SpiAccessViaSTM32F4<true, DmaChannelT>::BaudRatePrescaler_e p_prescaler = SpiAccessViaSTM32F4<true, DmaChannelT>::e_SpiPrescaler128)
        : SpiAccessViaSTM32F4<true, DmaChannelT>(reinterpret_cast<SPI_TypeDef *>(SpiT), p_txDmaChannel, p_rxDmaChannel, p_sclk, p_nsel, p_mosi, p_miso), m_rcc(p_rcc) {
            m_rcc.enable(SpiAccessViaSTM32F4RccFunction<SpiT>::m_type);
            SpiAccessViaSTM32F4GpioHelperT< SpiAccessViaSTM32F4GpioFunction<SpiT>::m_type >::initalize(p_sclk, p_nsel, p_mosi, p_miso);
            this->initialize(p_prescaler);
    };

    ~SpiAccessViaSTM32F4Fixed() {
        this->terminate();
        SpiAccessViaSTM32F4GpioHelperT< SpiAccessViaSTM32F4GpioFunction<SpiT>::m_type >::terminate(m_sclk, m_nsel, m_mosi, m_miso);
        m_rcc.disable(SpiAccessViaSTM32F4RccFunction<SpiT>::m_type);
    };
};

} /* namespace spi */

#endif /* _SPI_ACCESS_STM32F4_9355aeb1_b4d5_4a6b_a4e5_b332e730442b */
