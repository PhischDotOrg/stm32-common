/*
*****************************************************************************
**

**  File        : LinkerScript.ld
**
**  Abstract    : Linker script for STM32F407VGTx Device with
**                1024KByte FLASH, 128KByte RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**
**  Distribution: The file is distributed as is, without any warranty
**                of any kind.
**
**  (c)Copyright Ac6.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Ac6 permit registered System Workbench for MCU users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the System Workbench for MCU toolchain.
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

EXTERN (initStackSz stackSz)

PROVIDE(initStackSz = 0x80);
PROVIDE(freeRtosStackSz = 0x4000);

/* Highest address of the user mode stack */
estack = 0x2001FFFF;    /* end of RAM */
bstack = estack - freeRtosStackSz;

ucHeap = bstack;

eInitStack = ucHeap;
bInitStack = eInitStack - initStackSz;

/*
 * During startup, the main stack is placed in the region [bstack:estack].
 * The top-of-stack pointer is located through the estack symbol by the startup
 * assembly code in file 'startup_stm32f407xx.s'.
 *
 * The FreeRTOS Scheduler re-claims the main stack. It uses the NVIC offset
 * register to locate the address of the 'estack' symbol and uses the value stored
 * there to re-set the top-of-stack pointer. See the method 'prvPortStartFirstTask'
 * implemented in file 'port.c' for details.
 *
 * So in both cases, the top-of-stack is at address 'estack'.
 *
 * FreeRTOS manages the stacks of the running tasks. It does so by using a heap
 * implementation. The implementation locates the heap through the ucHeap symbol
 * in combination with the 'configTOTAL_HEAP_SIZE' config option. In our case
 * this is set to a size of 16kB.
 *
 * Hence, the stacks of the tasks are located in the region [0x2001c000-0x20001FFFF].
 */

/* Specify the memory areas */
MEMORY
{
ISR (r)		: ORIGIN = 0x08000000, LENGTH = 512
FIXEDDATA (r)	: ORIGIN = 0x08000400, LENGTH = 512
FLASH (rx)      : ORIGIN = 0x08001000, LENGTH = 1023K
RAM (xrw)      	: ORIGIN = 0x20000000, LENGTH = 128K
CCMRAM (rw)     : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
    . = ALIGN(4);
  } >ISR

  .fixeddata : {
    . = ORIGIN(FIXEDDATA);
    gFixedDataBegin = .;
    *(.fixeddata)
    gFixedDataUsed = .;
    gFixedDataEnd = ORIGIN(FIXEDDATA) + LENGTH(FIXEDDATA);
  } >FIXEDDATA

  /* The program code and other data goes into FLASH */
  .text :
  {
    . = ALIGN(4);
    stext = .;
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    . = ALIGN(4);
    _ctors_start_ = .;
    KEEP(*(.init_array));                 /* C++ constructors */
    KEEP(*(.ctors));                      /* C++ constructors */
    _ctors_end_ = .;

/*
    KEEP (*(.init))
    KEEP (*(.fini))
 */
    . = ALIGN(4);
    etext = .;         /* define a global symbols at end of code */
  } >FLASH

  /* Constant data goes into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

/* PhS, June 2nd 2015: Not sure what this is required for, so comment out for now
  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH
PhS, June 2nd 2015: Not sure what this is required for, so comment out for now */

  /* used by the startup to initialize data */
  _sidata = LOADADDR(.data);

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : 
  {
    . = ALIGN(4);
    sdata = .;         /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    edata = .;         /* define a global symbol at data end */
  } >RAM AT> FLASH

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    sbss = .;          /* define a global symbol at bss start */
    __bss_start__ = sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    ebss = .;          /* define a global symbol at bss end */
    __bss_end__ = ebss;
  } >RAM

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(4);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + freeRtosStackSz;
    . = . + initStackSz;
    . = ALIGN(4);
  } >RAM

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}


