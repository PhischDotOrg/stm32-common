/*-
 * $Copyright$
-*/

#ifndef _DMA_TYPES_STM32F4_HPP_2ae06011_8e2d_4019_a043_2756dccd4b9d
#define _DMA_TYPES_STM32F4_HPP_2ae06011_8e2d_4019_a043_2756dccd4b9d

namespace dma {

/*******************************************************************************
 * Definition corresponds to DIR Bits in DMA_SxCR Register
 ******************************************************************************/
typedef enum DmaDirection_e {
    e_PeripheralToMemory = 0,
    e_MemoryToPeripheral = 1,
    e_MemoryToMemory = 2,
    e_Undefined = 3
} DmaDirection_t;

/*******************************************************************************
 * Definition corresponds to MBURST/PBURST Bits in DMA_SxCR Register
 ******************************************************************************/
typedef enum DmaBurstSize_e {
    e_Single = 0,
    e_Incr4 = 1,
    e_Incr8 = 2,
    e_Incr16 = 3
} DmaBurstSize_t;

/*******************************************************************************
* Definition corresponds to FTH Bits in DMA_SxFCR Register
  ******************************************************************************/
typedef enum DmaFifoThreshold_e {
    e_Quarter = 0,
    e_Half = 1,
    e_ThreeQuarter = 2,
    e_Full = 3
} DmaFifoThreshold_t;

/*******************************************************************************
 * Definition corresponds to TCIF, HTIF, TEIF, DMEIF, FEIF Bits in DMA Interrupt
 * Status Registers (DMA_LISR, DMA_HISR)
 ******************************************************************************/
typedef enum DmaIrqStatus_e {
    e_FifoError         = 0x01,
    e_DirectModeError   = 0x04,
    e_TransferError     = 0x08,
    e_HalfTransfer      = 0x10,
    e_Complete          = 0x20
} DmaIrqStatus_t;

/*******************************************************************************
 *
 ******************************************************************************/
typedef enum DmaTransferStatus_e {
    e_DmaUndefined,
    e_DmaInProgress,
    e_DmaError,
    e_DmaComplete
} DmaTransferStatus_t;


} /* namespace dma */

#endif /* _DMA_TYPES_STM32F4_HPP_2ae06011_8e2d_4019_a043_2756dccd4b9d */
