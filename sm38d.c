#include "sm38d.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define MAXLN 32
char chbuf[MAXLN];

static int prop(int c, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(chbuf, MAXLN, fmt, args);
    va_end(args);
    return c;
}

static uint16_t read_word(const uint8_t* sm38)
{
    return (*sm38) | (*(sm38+1) << 8);
}

static const char* OPS[8] = {
    "B", "C", "D", "E", "H", "L", "(HL)", "A"
};

int sm38d(const uint8_t* sm38, const char** outchar)
{
    chbuf[0] = 0;
    *outchar = &chbuf[0];
    
    uint8_t op = *(sm38++);
    switch (op)
    {
    case 0x0:
        return prop(1, "NOP");
    
    case 0x01:
        return prop(3, "ld BC, $%04x", read_word(sm38));
        
    case 0x02:
        return prop(1, "ld (BC), A");
    
    case 0x03:
        return prop(1, "inc BC");
    
    case 0x04:
        return prop(1, "inc B");
        
    case 0x05:
        return prop(1, "dec B");
        
    case 0x06:
        return prop(2, "ld B, $%02X", *sm38);
    
    case 0x07:
        return prop(1, "rlca");
        
    case 0x08:
        return prop(3, "ld $%04x, (sp)", read_word(sm38));
    
    case 0x09:
        return prop(1, "add HL, BC");
    
    case 0x0A:
        return prop(1, "ld A, (BC)");
        
    case 0x0B:
        return prop(1, "dec BC");
    
    case 0x0C:
        return prop(1, "inc C");
    
    case 0x0D:
        return prop(1, "dec C");
        
    case 0x0E:
        return prop(2, "ld C, $%02x", *sm38);
    
    case 0x0F:
        return prop(1, "rrca");
        
    case 0x10:
        return prop(1, "stop");
    
    case 0x11:
        return prop(4, "ld DE, $%04x", read_word(sm38));
    
    case 0x12:
        return prop(1, "ld (DE), a");
        
    case 0x13:
        return prop(1, "inc DE");
    
    case 0x14:
        return prop(1, "inc D");
        
    case 0x15:
        return prop(1, "dec D");
    
    case 0x16:
        return prop(2, "ld D, $%02x", *sm38);
        
    case 0x17:
        return prop(1, "rla");
    
    case 0x18:
        return prop(2, "jr $%02x", *sm38); // TODO signed
    
    case 0x19:
        return prop(1, "add HL, DE");
        
    case 0x1A:
        return prop(1, "ld A, (DE)");
        
    case 0x1B:
        return prop(1, "dec DE");
    
    case 0x1C:
        return prop(1, "inc E");
    
    case 0x1D:
        return prop(1, "dec E");
        
    case 0x1E:
        return prop(2, "ld E, $%02x", *sm38);
        
    case 0x1F:
        return prop(1, "rra");
        
    case 0x20:
        return prop(2, "jr nz, $%02x", *sm38); // todo: relative
        
    case 0x21:
        return prop(3, "ld HL, $%04x", read_word(sm38));
    
    case 0x22:
        return prop(1, "ld (HL+), a");
    
    case 0x23:
        return prop(1, "inc HL");
        
    case 0x24:
        return prop(1, "inc H");
        
    case 0x25:
        return prop(1, "dec H");
    
    case 0x26:
        return prop(2, "dec (HL), $%02x", *sm38);
        
    case 0x27:
        return prop(1, "daa");
    
    case 0x28:
        return prop(2, "jr z, $%02x", *sm38);
    
    case 0x2A:
        return prop(1, "ld a, (HL+)");
        
    case 0x2B:
        return prop(1, "dec HL");
        
    case 0x2C:
        return prop(1, "inc L");
        
    case 0x2D:
        return prop(1, "dec L");
        
    case 0x2E:
        return prop(2, "ld L, $%02x", *sm38);
        
    case 0x2F:
        return prop(1, "cpl");
        
    case 0x30:
        return prop(2, "jr nc,$%02x", *sm38);
        
    case 0x31:
        return prop(3, "ld SP, $%04x", read_word(sm38));
    
    case 0x32:
        return prop(1, "ld (HL-), a");
        
    case 0x33:
        return prop(1, "inc SP");
        
    case 0x34:
        return prop(1, "inc (HL)");
        
    case 0x35:
        return prop(1, "dec (HL)");
    
    case 0x36:
        return prop(2, "ld HL, $%02x", *sm38);
        
    case 0x37:
        return prop(1, "scf");
        
    case 0x38:
        return prop(2, "jr c,$%02x", *sm38);
        
    case 0x39:
        return prop(1, "add HL, SP");
        
    case 0x3A:
        return prop(1, "ld A, (HL-)");
        
    case 0x3B:
        return prop(1, "dec SP");
        
    case 0x3C:
        return prop(1, "inc A");
        
    case 0x3D:
        return prop(1, "dec A");
        
    case 0x3E:
        return prop(1, "ld A, $%02x", *sm38);
        
    case 0x3F:
        return prop(1, "ccf");
        
    case 0x40 ... 0x75:
    case 0x77 ... 0x7F:
    
        switch (op / 0x8)
        {
            case 0x40/8:
                return prop(1, "ld B, %s", OPS[op%8]);
            case 0x48/8:
                return prop(1, "ld C, %s", OPS[op%8]);
            case 0x50/8:
                return prop(1, "ld D, %s", OPS[op%8]);
            case 0x58/8:
                return prop(1, "ld E, %s", OPS[op%8]);
            case 0x60/8:
                return prop(1, "ld H, %s", OPS[op%8]);
            case 0x68/8:
                return prop(1, "ld L, %s", OPS[op%8]);
            case 0x70/8:
                return prop(1, "ld (HL), %s", OPS[op%8]);
            case 0x78/8:
                return prop(1, "ld A, %s", OPS[op%8]);
        }
    
    case 0x76:
        return prop(1, "halt");
    
    case 0x80 ... 0xBF:
        switch (op)
        {
        case 0x80 ... 0x87:
                return prop(1, "add %s", OPS[op%8]);
            case 0x88 ... 0x8F:
                return prop(1, "adc %s", OPS[op%8]);
            case 0x90 ... 0x97:
                return prop(1, "sub %s", OPS[op%8]);
            case 0x98 ... 0x9F:
                return prop(1, "sbc %s", OPS[op%8]);
            case 0xA0 ... 0xA7:
                return prop(1, "and %s", OPS[op%8]);
            case 0xA8 ... 0xAF:
                return prop(1, "xor %s", OPS[op%8]);
            case 0xB0 ... 0xB7:
                return prop(1, "or %s", OPS[op%8]);
            case 0xB8 ... 0xBF:
                return prop(1, "cp %s", OPS[op%8]);
        }
    
    case 0xC0:
        return prop(1, "ret nz");
        
    case 0xC1:
        return prop(1, "pop BC");
        
    case 0xC2:
        return prop(3, "jp nz, $%04x", read_word(sm38));
        
    case 0xC3:
        return prop(3, "jp $%04x", read_word(sm38));
        
    case 0xC4:
        return prop(3, "call nz", read_word(sm38));
        
    case 0xC5:
        return prop(1, "push BC");
        
    case 0xC6:
        return prop(2, "add A, $%02x", *sm38);
        
    case 0xC7:
        return prop(1, "rst 0");
        
    case 0xC8:
        return prop(1, "ret z");
        
    case 0xC9:
        return prop(1, "ret");
        
    case 0xCA:
        return prop(3, "jmp z, $%04x", read_word(sm38));
        
    case 0xCB:
        {
            uint8_t opx = *sm38;
            uint8_t arg = opx%8;
            switch(opx/8)
            {
            case 0x00/8:
                return prop(2, "RLC %s", OPS[arg]);
            case 0x08/8:
                return prop(2, "RRC %s", OPS[arg]);
            case 0x10/8:
                return prop(2, "ROL %s", OPS[arg]);
            case 0x18/8:
                return prop(2, "ROR %s", OPS[arg]);
            case 0x20/8:
                return prop(2, "SLA %s", OPS[arg]);
            case 0x28/8:
                return prop(2, "SRA %s", OPS[arg]);
            case 0x30/8:
                return prop(2, "SWAP %s", OPS[arg]);
            case 0x38/8:
                return prop(2, "SRL %s", OPS[arg]);
            case 0x40/8 ... 0x78/8:
                return prop(2, "BIT %d, %s", opx/8 - 0x40/8, OPS[arg]);
            case 0x80/8 ... 0xB8/8:
                return prop(2, "RES %d, %s", opx/8 - 0x80/8, OPS[arg]);
            case 0xC0/8 ... 0xF8/8:
                return prop(2, "SET %d, %s", opx/8 - 0xC0/8, OPS[arg]);
            }
        }
    
    case 0xCC:
        return prop(3, "call z, $%04x", read_word(sm38));
        
    case 0xCD:
        return prop(3, "call $%04x", read_word(sm38));
        
    case 0xCE:
        return prop(2, "adc $%02x", *sm38);
        
    case 0xCF:
        return prop(1, "rst 1");
        
    case 0xD0:
        return prop(1, "ret nc");
        
    case 0xD1:
        return prop(1, "pop DE");
        
    case 0xD2:
        return prop(3, "jp nc, $%04x", read_word(sm38));
        
    case 0xD4:
        return prop(3, "call nc", read_word(sm38));
        
    case 0xD5:
        return prop(1, "push DE");
        
    case 0xD6:
        return prop(2, "sub A, $%02x", *sm38);
        
    case 0xD7:
        return prop(1, "rst 2");
        
    case 0xD8:
        return prop(1, "ret c");
        
    case 0xD9:
        return prop(1, "reti");
        
    case 0xDA:
        return prop(3, "jmp c, $%04x", read_word(sm38));
    
    case 0xDC:
        return prop(3, "call c, $%04x", read_word(sm38));
        
    case 0xDE:
        return prop(2, "sbc $%02x", *sm38);
        
    case 0xDF:
        return prop(1, "rst 3");
        
    case 0xE0:
        return prop(2, "ld ($%02x), A", *sm38);
        
    case 0xE1:
        return prop(1, "pop HL");
        
    case 0xE2:
        return prop(1, "ld (C), A");
        
    case 0xE5:
        return prop(1, "push HL");
        
    case 0xE6:
        return prop(2, "and $%02x", *sm38);
        
    case 0xE7:
        return prop(1, "rst 4");
        
    case 0xE8:
        return prop(2, "add SP, $%02x", *sm38);
        
    case 0xE9:
        return prop(1, "jmp HL");
        
    case 0xEA:
        return prop(3, "ld ($%04x), a", read_word(sm38));
        
    case 0xEE:
        return prop(2, "xor $%02x", *sm38);
        
    case 0xEF:
        return prop(1, "rst 5");
        
    case 0xF0:
        return prop(2, "ld A, ($%02x)", *sm38);
        
    case 0xF1:
        return prop(1, "pop AF");
        
    case 0xF2:
        return prop(1, "ld A, (C)");
        
    case 0xF3:
        return prop(1, "di");
        
    case 0xF5:
        return prop(1, "push AF");
        
    case 0xF6:
        return prop(2, "or $%02x", *sm38);
        
    case 0xF7:
        return prop(1, "rst 6");
        
    case 0xF8:
        return prop(2, "ld HL, SP+$%02x", *sm38);
        
    case 0xF9:
        return prop(1, "LD SP, HL");
        
    case 0xFA:
        return prop(3, "ld A, ($%04x)", read_word(sm38));
        
    case 0xFB:
        return prop(1, "ei");
    
    case 0xFE:
        return prop(2, "CP $%02x", *sm38);
    
    case 0xFF:
        return prop(1, "rst 7");
    }
    
    return prop(1, "db $%02x", op);
}