/* Memory Map */

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_sp = ORIGIN(RAM) + LENGTH(RAM);	/* end of "RAM" Ram type memory */

_Min_Heap_Size = 0x200;	/* required amount of heap  */
_Min_Stack_Size = 0x2000;	/* required amount of stack */

/* Memories definition */
MEMORY
{
    FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 3840K
    AXISRAM (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
    ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
    DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    SRAM0 (xrw)   : ORIGIN = 0x30000000, LENGTH = 16K
    SRAM1 (xrw)   : ORIGIN = 0x30004000, LENGTH = 16K
}

/* Sections */
SECTIONS
{
    /* The startup code into "FLASH" Rom type memory */
    .vectors :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector)) /* Startup code */
        . = ALIGN(4);
    } >FLASH

    /* The program code and other data into "FLASH" Rom type memory */
    .text :
    {
        . = ALIGN(4);
        *(.text)           /* .text sections (code) */
        *(.text*)          /* .text* sections (code) */
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        . = ALIGN(4);
        _etext = .;        /* define a global symbols at end of code */
    } >FLASH

    /* Constant data into "FLASH" Rom type memory */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)         /* .rodata sections (constants, strings, etc.) */
        *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
        . = ALIGN(4);
    } >FLASH

    .ARM.extab (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
    {
        . = ALIGN(4);
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        . = ALIGN(4);
    } >FLASH

    .ARM (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
    {
        . = ALIGN(4);
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
        . = ALIGN(4);
    } >FLASH

    .preinit_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array*))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .init_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array*))
        PROVIDE_HIDDEN (__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .fini_array (READONLY) : /* The READONLY keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(SORT(.fini_array.*)))
        KEEP (*(.fini_array*))
        PROVIDE_HIDDEN (__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    /* Used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data sections into "AXISRAM" Ram type memory */
    .data : 
    {
        . = ALIGN(4);
        _sdata = .;        /* create a global symbol at data start */
        *(.data)           /* .data sections */
        *(.data*)          /* .data* sections */

        . = ALIGN(4);
        _edata = .;        /* define a global symbol at data end */

    } >AXISRAM AT> FLASH

    /* Uninitialized data section into "AXISRAM" Ram type memory */
    . = ALIGN(4);
    .bss :
    {
        /* This is used by the startup in order to initialize the .bss section */
        _sbss = .;         /* define a global symbol at bss start */
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        _ebss = .;         /* define a global symbol at bss end */
        __bss_end__ = _ebss;
    } >AXISRAM

    /* User_heap_stack section, used to check that there is enough "AXISRAM" Ram  type memory left */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + _Min_Heap_Size;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >AXISRAM

    /* Used by the startup to initialize itcmram */
    _siitcmram = LOADADDR(.itcmram);
    
    /* Initialized itcmram sections into "ITCMRAM" Ram type memory */
    .itcmram :
    {
        . = ALIGN(4);
        .sitcmram = .;        /* create a global symbol at itcmram start */
        *(.itcmram)           /* .itcmram sections */
        *(.itcmram*)          /* .itcmram* sections */

        . = ALIGN(4);
        _eitcmram = .;        /* define a global symbol at itcmram end */

    } >ITCMRAM AT> FLASH

    /* Used by the startup to initialize dtcmram */
    _sidtcmram = LOADADDR(.dtcmram);

    /* Initialized dtcmram sections into "DTCMRAM" Ram type memory */
    .dtcmram :
    {
        . = ALIGN(4);
        .sdtcmram = .;        /* create a global symbol at dtcmram start */
        *(.dtcmram)           /* .dtcmram sections */
        *(.dtcmram*)          /* .dtcmram* sections */

        . = ALIGN(4);
        _edtcmram = .;        /* define a global symbol at dtcmram end */

    } >DTCMRAM AT> FLASH

    /* Used by the startup to initialize sram0 */
    _sisram0 = LOADADDR(.sram0);

    /* Initialized sram0 sections into "SRAM0" Ram type memory */
    .sram0 :
    {
        . = ALIGN(4);
        .ssram0 = .;        /* create a global symbol at sram0 start */
        *(.sram0)           /* .sram0 sections */
        *(.sram0*)          /* .sram0* sections */

        . = ALIGN(4);
        _esram0 = .;        /* define a global symbol at sram0 end */

    } >SRAM0 AT> FLASH

    /* Used by the startup to initialize sram1 */
    _sisram1 = LOADADDR(.sram1);

    /* Initialized sram1 sections into "SRAM1" Ram type memory */
    .sram1 :
    {
        . = ALIGN(4);
        .ssram1 = .;        /* create a global symbol at sram1 start */
        *(.sram1)           /* .sram1 sections */
        *(.sram1*)          /* .sram1* sections */

        . = ALIGN(4);
        _esram1 = .;        /* define a global symbol at sram1 end */

    } >SRAM1 AT> FLASH

    /* Remove information from the compiler libraries */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
