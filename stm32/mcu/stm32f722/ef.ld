/*
 * ST32F722 memory setup.
 */

MEMORY
{
    flash   : org = 0x00200000, len = 512k
    lflash  : org = 0x08000000, len = 512k
    ram0    : org = 0x20000000, len = 64k  /* DTCM-RAM */
    ram	    : org = 0x20010000, len = 176k
    ram1    : org = 0x2003c000, len = 16k
}

__ram0_start__          = ORIGIN(ram0);
__ram0_size__           = 64k;
__ram0_end__            = __ram0_start__ + __ram0_size__;
__ram_start__           = ORIGIN(ram);
__ram_size__            = 176k;
__ram_end__             = __ram_start__ + __ram_size__;
__ram1_start__           = ORIGIN(ram1);
__ram1_size__            = 16k;
__ram1_end__             = __ram1_start__ + __ram1_size__;

SECTIONS
{
    . = 0;

    .startup : ALIGN(4) SUBALIGN(4)
    {
	_startup = .;
        KEEP(*(.startup.vectors))
	. = ALIGN(1024);
	KEEP(*(.startup.entry))
	. = ALIGN(1024);
    } > flash AT > lflash

    _text = .;
    .text : ALIGN(16) SUBALIGN(16)
    {
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
	
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
	. = ALIGN(8);
    } > flash AT > lflash

    .ARM.extab : {*(.ARM.extab* .gnu.linkonce.armextab.*)} > flash AT > lflash

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash AT > lflash

    .eh_frame_hdr : {*(.eh_frame_hdr)} > flash AT > lflash

    .eh_frame : ONLY_IF_RO {*(.eh_frame)} > flash AT > lflash

    .init_array     :
    {   
        __init_array_start = .;
    	KEEP (*(SORT(.init_array.*)))
	KEEP (*(.init_array ))
        __init_array_end = .;
	. = ALIGN(8);
    } > flash AT > lflash

    .fini_array     :
    {
	__fini_array_start = .; 
	KEEP (*(.fini_array))
	KEEP (*(SORT(.fini_array.*)))
	__fini_array_end = .; 
    } > flash AT > lflash

    _etext = .;
    _textdata = _etext;

    .vectors :
    {
        . = ALIGN(8);
        __vector_ram_addr__ = .;
	KEEP(*(.vectors))
    } > ram

    .data :
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        . = ALIGN(4);
        *(.data.*)
        . = ALIGN(4);
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > ram AT > lflash

    .bss :
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        . = ALIGN(4);
        *(.bss.*)
        . = ALIGN(4);
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
    } > ram0

    .ram1 (NOLOAD) :
    {
        . = ALIGN(4);
	*(.ram1)    
        . = ALIGN(4);
    } > ram1

    PROVIDE(end = .);
    _end            = .;
}

__initial_sp__ = __ram0_end__ - 256;
__heap_base__     = _edata;
__heap_end__      = __ram_end__;
