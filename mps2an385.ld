ENTRY(_start)

MEMORY
{
    FLASH : ORIGIN = 0x0, LENGTH = 0x4000
    RAM : ORIGIN = 0x21000000, LENGTH = 16M
}

_Min_Heap_Size = 8M;
_Min_Stack_Size = 2M;

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > FLASH

    .text :
    {
        *(.text)
        *(.text*)
    } > RAM

    .rodata :
    {
        *(.rodata)
        *(.rodata*)
    } > RAM

    .data :
    {
        _sdata = .;
        *(.data)
        *(.data*)
        _edata = .;
    } > RAM

    .bss :
    {
        _sbss = .;
        PROVIDE(__bss_start__ = .);
        *(.bss)
        *(.bss*)
        _ebss = .;
        PROVIDE(__bss_end__ = .);
    } > RAM
    
    .heap :
    {
        __end__ = .;
        PROVIDE( end = . );
        PROVIDE( _heap_start = end );
        . = . + _Min_Heap_Size;
        PROVIDE(_heap_end = .);
    } > RAM
    
    .stack :
    {
        . = . + _Min_Stack_Size;
        PROVIDE(__stack = .);
    } > RAM

    /DISCARD/ :
    {
        *(.comment)
    }
}
