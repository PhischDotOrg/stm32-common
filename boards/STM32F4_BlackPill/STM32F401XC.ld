/* Entry Point */
ENTRY(Reset_Handler)

/*
 * During startup, the main stack is placed in the region [bstack:estack].
 * The top-of-stack pointer is located through the estack symbol by the startup
 * assembly code in CPU-specific 'startup_stm32*.s' file.
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
 * in combination with the 'configTOTAL_HEAP_SIZE' config option. In our case,
 * the ucHeap Symbol is placed in the region 'HEAP'.
 *
 * Hence, the stacks of the tasks are located in the region 'HEAP' defined below.
 */

/* Specify the memory areas */
MEMORY
{
  ISR (r)       : ORIGIN = 0x08000000, LENGTH = 512
  FIXEDDATA (r) : ORIGIN = 0x08000400, LENGTH = 512
  FLASH (rx)    : ORIGIN = 0x08001000, LENGTH = 255K
  STACK (rw)    : ORIGIN = 0x20000000, LENGTH = 4K
  /* Make sure the length of this section matches the definition of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h */
  HEAP (rw)     : ORIGIN = 0x20001000, LENGTH = 4K
  RAM (rw)      : ORIGIN = 0x20002000, LENGTH = 56K
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

  .stack :
  {
    . = ORIGIN(STACK);
    . = ALIGN(4);
    /* Bottom of Stack */
    bstack = .;
    
    . = ORIGIN(STACK) + LENGTH(STACK);
    . = ALIGN(4);
    /* Top of Stack */
    estack = .;
  } >STACK

  .heap :
  {
    . = ORIGIN(HEAP);
    . = ALIGN(4);

    /* FreeRTOS Heap */
    ucHeap = .;

    . = ORIGIN(HEAP) + LENGTH(HEAP);
    . = ALIGN(4);
  } >HEAP

  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
