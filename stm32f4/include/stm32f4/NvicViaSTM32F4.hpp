/*-
 * $Copyright$
-*/

#ifndef _NVIC_STM32F4_HPP_f721335c_757e_4618_a7f1_d39f1820967f
#define _NVIC_STM32F4_HPP_f721335c_757e_4618_a7f1_d39f1820967f

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include <stm32f4xx.h>

#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

#include <stm32f4/ScbViaSTM32F4.hpp>
#include <stm32f4/AdcViaSTM32F4.hpp>
#include <dma/DmaStreamViaSTM32F4.hpp>
#include <spi/SpiAccessViaSTM32F4.hpp>
#include <timer/TimerViaSTM32F4.hpp>
#include <usb/UsbCore.hpp>

namespace devices {

/*******************************************************************************
 * 
 ******************************************************************************/
class NvicViaSTM32F4Base {
public:
    typedef enum Irq_e {
        /******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
        NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
        MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
        BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
        UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
        SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
        DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
        PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
        SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
        /******  STM32 specific Interrupt Numbers **********************************************************************/
        WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
        PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
        TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
        RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
        FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
        RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
        EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
        EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
        EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
        EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
        EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
        DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
        DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
        DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
        DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
        DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
        DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
        DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
        ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
        CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
        CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
        CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
        CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
        EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
        TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
        TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
        TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
        TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
        TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
        TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
        TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
        I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
        I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
        I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
        I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */  
        SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
        SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
        USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
        USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
        USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
        EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
        RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
        OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */    
        TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
        TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
        TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
        TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
        DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
        FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
        SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
        TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
        SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
        UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
        UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
        TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
        TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
        DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
        DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
        DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
        DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
        DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
        ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
        ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
        CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
        CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
        CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
        CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
        OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
        DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
        DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
        DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
        USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */ 
        I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
        I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
        OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
        OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
        OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
        OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
        DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
        CRYP_IRQn                   = 79,     /*!< CRYP crypto global interrupt                                      */
        HASH_RNG_IRQn               = 80,      /*!< Hash and Rng global interrupt                                     */
        FPU_IRQn                    = 81      /*!< FPU global interrupt                                              */
    } Irq_t;

    void disableIrq(const Irq_t p_irq) const;

protected:
    NvicViaSTM32F4Base(NVIC_Type *p_nvic);
    ~NvicViaSTM32F4Base(void);

    static const Irq_t m_dmaIrq[2][8];
    
    void enableIrqWithPriorityGroup(const Irq_t p_irq, const unsigned p_priorityGroup) const;
    
private:
    NVIC_Type * const m_nvic;
};

/*******************************************************************************
 * 
 ******************************************************************************/
template<typename T> struct NvicViaSTM32F4_IrqHelper;

template<> struct NvicViaSTM32F4_IrqHelper<devices::AdcViaSTM32F4_Adc1> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::ADC_IRQn;
};

template<> struct NvicViaSTM32F4_IrqHelper<spi::SpiAccessViaSTM32F4_Spi1> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::SPI1_IRQn;
};

template<> struct NvicViaSTM32F4_IrqHelper<spi::SpiAccessViaSTM32F4_Spi2> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::SPI2_IRQn;
};

template<> struct NvicViaSTM32F4_IrqHelper<timer::TimerViaSTM32F4_Tim2> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::TIM2_IRQn;
};

template<> struct NvicViaSTM32F4_IrqHelper<timer::TimerViaSTM32F4_Tim3> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::TIM3_IRQn;
};

#if defined(USB_OTG_FS_PERIPH_BASE)
template<> struct NvicViaSTM32F4_IrqHelper<usb::stm32f4::UsbFullSpeedCore> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::OTG_FS_IRQn;
};
#endif /* defined(USB_OTG_FS_PERIPH_BASE) */
#if defined(USB_OTG_HS_PERIPH_BASE)
template<> struct NvicViaSTM32F4_IrqHelper<usb::stm32f4::UsbHighSpeedCore> {
    static const NvicViaSTM32F4Base::Irq_t m_irq = NvicViaSTM32F4Base::OTG_HS_IRQn;
};
#endif /* defined(USB_OTG_HS_PERIPH_BASE) */

/*******************************************************************************
 *
 ******************************************************************************/
template<typename ScbT>
class NvicViaSTM32F4T : public NvicViaSTM32F4Base {
public:
    NvicViaSTM32F4T(NVIC_Type *p_nvic, ScbT &p_scb);
    ~NvicViaSTM32F4T(void);

    void enableIrq(const dma::DmaStreamViaSTM32F4 &p_obj) const;    
    void disableIrq(const dma::DmaStreamViaSTM32F4 &p_obj) const;    
    
    template<typename T>
    void enableIrq(const T & /* p_obj */) const {
        this->enableIrqWithPriorityGroup(NvicViaSTM32F4_IrqHelper<T>::m_irq);
    }

    template<typename T>
    void disableIrq(const T & /* p_obj */) const {
        NvicViaSTM32F4Base::disableIrq(NvicViaSTM32F4_IrqHelper<T>::m_irq);
    }
    
private:
    ScbT &m_scb;
    
    void enableIrqWithPriorityGroup(const Irq_t p_irq) const {
        NvicViaSTM32F4Base::enableIrqWithPriorityGroup(p_irq, static_cast<const unsigned>(this->m_scb.getPriorityGroup()));
    }
}; /* class NvicViaSTM32F4 */

typedef NvicViaSTM32F4T<devices::ScbViaSTM32F4> NvicViaSTM32F4;

} /* devices */

// #include "NvicViaSTM32F4.cpp"

#endif /* _NVIC_STM32F4_HPP_f721335c_757e_4618_a7f1_d39f1820967f */
