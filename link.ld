ENTRY(_reset) /* entry point */

MEMORY {
    flash(rx) : ORIGIN = 0x08000000, LENGTH = 64k
    sram(rwx) : ORIGIN = 0x20000000, LENGTH = 16k
}

_estack     = ORIGIN(sram) + LENGTH(sram); /* stack pointer to SRAM end */

SECTIONS {
    .vectors : { KEEP(*(.vectors)) } > flash
    .text : { *(.text*) }            > flash
    .rodata : { *(.rodata*) }        > flash

    .data : {
        _sdata = .;
        *(.first_data)
        *(.data SORT(.data.*))
        _edata = .;
    } > sram AT > flash
    _sidata = LOADADDR(.data);

    .bss : {
        _sbss = .;
        *(.bss SORT(.bss.*) COMMON)
        _ebss = .;
    } > sram

    . = ALIGN(8);
}