.define RUNTIME_INCLUDED 1
.define CUSTOM_RESET 1

reset:
    nop
    call main
exit:
     stop
     
.org $80
print_char_nocrc:
     ret

.org $100
     jp reset

.org $200
     
.memoryMap
     defaultSlot 0
     slot 0 $0000 size $4000
     slot 1 $C000 size $4000
.endMe

.romBankSize   $4000 ; generates $8000 byte ROM
.romBanks      2

.ifndef bss
     ; address of next normal variable
     .define bss    $D800
.endif

.ifndef dp
     ; address of next direct-page ($FFxx) variable
     .define dp     $FF80
.endif

.redefine bss  bss+1

; Stack is normally here
.define std_stack $DFFF

; Common routines
.include "macros.inc"
.include "delay.s"
.include "crc.s"
.include "printing.s"
.include "numbers.s"
.include "testing.s"