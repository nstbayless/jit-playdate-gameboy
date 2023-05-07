#include "armd.h"

#include <stddef.h>
#include <map.h>
#include <string.h>
#include <stdio.h>

#define MAXLN 64
static char chbuf[MAXLN];

//#define snprintf(...) do {} while (0)

#define error(...) do { snprintf(chbuf, MAXLN, "ERR: " __VA_ARGS__); return 0; } while(0)
#define assert(x) do { if (!(x)) error("Assertion Failed: " #x); } while (0)

static int decode_32(const uint32_t arm, uintptr_t base);
static int decode_16(const uint16_t arm, uintptr_t base);

#define SSFLAG(setflags) (setflags ? "S" : "")

#define MACRO_mx 0
#define MACRO_m0 1
#define MACRO_m1 1
#define MACRO_vx 0
#define MACRO_v0 0
#define MACRO_v1 1

#define MACRO_mg(a) MACRO_m ## a
#define MACRO_vg(a) MACRO_v ## a

#define BITSHIFT_MASK(a, b) ((a << 1) | MACRO_mg(b))
#define BITSHIFT_VALUE(a, b) ((a << 1) | MACRO_vg(b))

#define BITMATCH(a, ...) \
    ((a & (FOLD(BITSHIFT_MASK, 0, __VA_ARGS__))) \
    == FOLD(BITSHIFT_VALUE, 0, __VA_ARGS__))

#define ptr_to_uint(x) ((uintptr_t)(void*)x)

const uint16_t* armd(const uint16_t* arm, const char** out)
{
    *out = chbuf;
    chbuf[0] = 0;
    
    if ((*arm) >> 11 == 0b11101 || (*arm) >> 11 == 0b11111 || (*arm) >> 11 == 0b11110)
    {
        if (!decode_32(arm[1] | (arm[0] << 16), ptr_to_uint(arm)))
        {
            strcpy(chbuf, "UNK32");
        }
        
        return arm+2;
    }
    else
    {
        if (!decode_16(*arm, ptr_to_uint(arm)))
        {
            strcpy(chbuf, "UNK16");
        }
        
        return arm+1;
    }
    
    return NULL;
}

#define BUFFS
#define IMMAX 32
#define NBUFFS 8
static char buff_intermediate[NBUFFS][IMMAX];
static char* getbuff(void)
{
    static uint8_t i = 0;
    char* buff = buff_intermediate[(i++)%NBUFFS];
    buff[0] = 0;
    return buff;
}

static const char* regnames[16] = {
    "r0", "r1", "r2", "r3",
    "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "r11",
    "r12", "sp", "lr", "pc",
};

static const char* regnames_comma[16] = {
    "r0, ", "r1, ", "r2, ", "r3, ", "r4, ",
    "r5, ", "r6, ", "r7, ", "r8, ", "r9, ",
    "r10, ", "r11, ", "r12, ", "sp, ",
    "lr, ", "pc, "
};

static const char* rname(uint8_t reg)
{
    return regnames[reg];
}

static uint32_t bits(uint32_t src, uint32_t lowbit, uint32_t bitc)
{
    return (src >> lowbit) & ((1 << bitc) - 1);
}

static uint32_t bit(uint32_t src, uint32_t idx)
{
    return bits(src, idx, 1);
}

#define OPREGSEP(i, v) ((v) ? regnames_comma[i] : "")
#define OPREGSEPM(i) OPREGSEP(i, (regs >> i) & 1)
#define OPREGSEP8 MAP_LIST(OPREGSEPM, 0, 1, 2, 3, 4, 5, 6, 7)
#define OPREGSEP16 MAP_LIST(OPREGSEPM, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15)

#define IMM5_RD_RM(u) ((u >> 6) & 0x1F), (u & 0xF), ((u >> 4)&0xF)

const char* expand_imm12c(uint16_t imm12)
{
    char* buff = getbuff();
    imm12 &= 0x0fff;
    if (imm12 >> 10 == 0)
    {
        unsigned int imm8 = bits(imm12, 0, 8);
        switch (bits(imm12, 8, 2))
        {
        case 0b00:
            snprintf(buff, IMMAX, "#$%02x", imm8);
            break;
        case 0b01:
            if (imm8 == 0)
            {
                snprintf(buff, IMMAX, "#$??");
            }
            else
            {
                snprintf(buff, IMMAX, "#$00%02x00%02x", imm8, imm8);
            }
            break;
        case 0b10:
            if (imm8 == 0)
            {
                snprintf(buff, IMMAX, "#$??");
            }
            else
            {
                snprintf(buff, IMMAX, "#$%02x00%02x00", imm8, imm8);
            }
            break;
        case 0b11:
            if (imm8 == 0)
            {
                snprintf(buff, IMMAX, "#$??");
            }
            else
            {
                snprintf(buff, IMMAX, "#$%02x%02x%02x%02x", imm8, imm8, imm8, imm8);
            }
            break;
        }
    }
    else
    {
        unsigned int unrot = bits(imm12, 0, 7);
        unsigned int rot = bits(imm12, 7, 4);
        snprintf(buff, IMMAX, "#RORC(%02x, %01x)", unrot, rot);
    }
    
    return buff;
}

const char* regshift(uint8_t reg, uint8_t shift_type, uint8_t imm5)
{
    char* buf = getbuff();
    
    switch(shift_type & 3)
    {
    case 0:
        if (imm5 == 0)
        {
            return rname(reg);
        }
        snprintf(buf, IMMAX, "%s<<%d", rname(reg), imm5);
        break;
    case 1:
        if (imm5 == 0) imm5 = 32;
        snprintf(buf, IMMAX, "%s>>>%d", rname(reg), imm5);
        break;
    case 2:
        if (imm5 == 0) imm5 = 32;
        snprintf(buf, IMMAX, "%s>>%d", rname(reg), imm5);
        break;
    case 3:
        if (imm5 == 0)
            snprintf(buf, IMMAX, "C>%s>C", rname(reg));
        else
            snprintf(buf, IMMAX, "%s>ROR>%d", rname(reg), imm5);
        break;
    }
    
    return buf;
}

static int decode_binop_16_rm_rdn(const char* name, uint16_t arm)
{
    uint8_t rdn = bits(arm, 0, 3);
    uint8_t rm = bits(arm, 3, 3);
    snprintf(chbuf, MAXLN, "%sS %s, %s", name, rname(rdn), rname(rm));
    return 1;
}

static int decode_op32_rn_rd_imm12c(const char* name, uint32_t arm, int rnoff)
{
    uint8_t rd = bits(arm, 8, 4);
    uint8_t rn = bits(arm, 16+rnoff, 4);
    uint16_t imm12 = bits(arm, 0, 8) | (bits(arm, 12, 3) << 8) | (bit(arm, 26) << 11);
    uint8_t setflags = bit(arm, 20);
    if (rd == 13 || rd == 15 || rn == 13) return 0;
    snprintf(chbuf, MAXLN, "ORR%s %s,%s,%s", SSFLAG(setflags), rname(rd), rname(rn), expand_imm12c(imm12));
    return 1;
}

static int decode_op32_rd_rn_rmshift(const char* name, uint32_t arm)
{
    uint8_t rn = bits(arm, 16, 4);
    uint8_t rm = arm & 0xF;
    uint8_t rd = bits(arm, 8, 4);
    uint8_t imm5 = bits(arm, 6, 2) | (bits(arm, 12, 3) << 2);
    uint8_t type = bits(arm, 4, 2);
    uint8_t setflags = bit(arm, 20);
    
    if (rd == 15 && setflags) return 0;
    if (rd == 13 || rd == 15 || rn == 13 || rn == 15 || rm == 13 || rm == 15) return 0;
    
    snprintf(chbuf, MAXLN, "%s%s.W %s, %s, %s", name, SSFLAG(setflags), rname(rd), rname(rn), regshift(rm, type, imm5));
    return 1;
}

static int decode_imm_shift(uint8_t shift_type, uint8_t imm5, uint8_t rd, uint8_t rm)
{
    switch(shift_type & 3)
    {
    case 0:
        snprintf(chbuf, MAXLN, "LSL %s, %s, %d", rname(rd), rname(rm), imm5);
        break;
    case 1:
        snprintf(chbuf, MAXLN, "LSR %s, %s, %d", rname(rd), rname(rm), imm5 == 0 ? 32 : imm5);
        break;
    case 2:
        snprintf(chbuf, MAXLN, "ASR %s, %s, %d", rname(rd), rname(rm), imm5 == 0 ? 32 : imm5);
        break;
    case 3:
        if (imm5 == 0)
            snprintf(chbuf, MAXLN, "RRX %s, %s, 1", rname(rd), rname(rm));
        else
            snprintf(chbuf, MAXLN, "ROR %s, %s, %d", rname(rd), rname(rm), imm5);
        break;
    }
    
    return 1;
}

static int decode_32(const uint32_t arm, uintptr_t base)
{
    // A5-13
    assert(bits(arm, 29, 3) == 0b111);
    uint32_t opcode = (bits(arm, 20, 9) << 1) | bit(arm, 15);
    
    if      (BITMATCH(opcode, 0, 1,   0, 0, x, x,  0, x, x,   x))
    {
        // A5-20 load/store multiple
        uint32_t op = (bits(arm, 23, 2) << 1) | bit(arm, 20);
        int is_alt_behaviour = (bits(arm, 16, 4) | (bit(arm, 21) << 1)) == 0b11101;
        
        switch(op)
        {
        case 0b010:
            // A6-218 store multiple
            break;
        case 0b011:
            if (!is_alt_behaviour)
            {
                // A6-84 load multiple
            }
            else
            {
                // A6-186 pop
                uint8_t p = bit(arm, 15);
                uint8_t m = bit(arm, 14);
                uint16_t regs = bits(arm, 0, 12) | (p << 15) | (m << 14);
                if (p && m) return 0;
                if (__builtin_popcount(regs) <= 1) return 0;
                
                snprintf(chbuf, MAXLN, "POP { %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s }",
                    OPREGSEP16
                );
            }
            break;
        case 0b100:
            if (!is_alt_behaviour)
            {
                // A6-220 store multiple
            }
            else
            {
                // A6-188 push
            }
            break;
        case 0b101:
            // A6-286 load multiple
            break;
        }
    }
    else if (BITMATCH(opcode, 0, 1,   0, 0, x, x,  1, x, x,   x))
    {
        // A5-21 load/store dual/exclusive
    }
    else if (BITMATCH(opcode, 0, 1,   0, 1, x, x,  x, x, x,   x))
    {
        // A5-26 data processing (shifted register)
        // all operands are (reg)
        int s = !!(arm & 0x21);
        uint8_t op = (arm >> 22) & 0xF;
        uint8_t rd = (arm >> 8) & 0xF;
        uint8_t rn = (arm >> 16) & 0xF;
        
        switch(op)
        {
        case 0b0000:
            if (s == 1)
            {
                // A6-264 TST
            }
            else if (rd != 0xF)
            {
                // A6-34 AND (reg)
                return decode_op32_rd_rn_rmshift("AND", arm);
            }
            break;
        case 0b0001:
            // A6-46 BIC
            break;
        case 0b0010:
            if (rn != 0xF)
            {
                // A6-174 ORR (reg)
                return decode_op32_rd_rn_rmshift("ORR", arm);
            }
            else
            {
                // A5-27 mov imm shift
            }
            break;
        case 0b0011:
            if (rn != 0xF)
            {
                // A6-170 ORN
            }
            else
            {
                // A6-164 MVN
            }
            break;
        case 0b0100:
            if (rd != 0xF)
            {
                // A6-74 EOR (reg)
                return decode_op32_rd_rn_rmshift("EOR", arm);
            }
            else if (s == 1)
            {
                // A6-261 TEQ
            }
            break;
        case 0b1000:
            if (rd != 0xF)
            {
                // A6-24 ADD
            }
            else if (s == 1)
            {
                // A6-60 CMN
            }
            break;
        case 0b1010:
            // A6-20 ADC
            break;
        case 0b1011:
            // A6-206 SBC
            break;
        }
    }
    else if (BITMATCH(opcode, 0, 1,   1, x, x, x,  x, x, x,   x))
    {
        // A5-32 coprocessor
    }
    else if (BITMATCH(opcode, 1, 0,   x, 0, x, x,  x, x, x,   0))
    {
        // data processing (modified immediate) A5-14
        // all operands are (imm).
        uint8_t op = (arm >> 21) & 0xF;
        uint8_t rd = (arm >> 8) & 0xF;
        uint8_t rn = (arm >> 20) & 0xF;
        
        switch (op)
        {
        case 0b0000:
            if (rd != 0xF)
            {
                // A6-32 AND (imm)
                return decode_op32_rn_rd_imm12c("AND", arm, 0);
            }
            else
            {
                // A6-262 TST
            }
            break;
        case 0b0001:
            // A6-44 BIC
            break;
        case 0b0010:
            if (rn != 0xF)
            {
                // A6-172 ORR (imm)
                return decode_op32_rn_rd_imm12c("ORR", arm, 0);
            }
            else
            {
                // A6-148 MOV
                
            }
            break;
        case 0b0011:
            if (rn != 0xF)
            {
                // A6-168 ORN (imm)
                return decode_op32_rn_rd_imm12c("ORN", arm, 0);
            }
            else
            {
                // A6-162 MVN
            }
            break;
        case 0b0100:
            if (rd != 0xF)
            {
                // A6-72 EOR
                return decode_op32_rn_rd_imm12c("EOR", arm, 1);
            }
            else
            {
                // A6-260 TEQ
            }
            break;
        case 0b1000:
            if (rd != 0xF)
            {
                // A6-22 ADD
            }
            else
            {
                // A6-58 CMN
            }
            break;
        case 0b1010:
            // A6-18 ADC
            break;
        case 0b1011:
            // A6-204 SBC
            break;
        case 0b1101:
            if (rd != 0xF)
            {
                // A6-244 SUB
            }
            else
            {
                // A6-62 CMP
            }
            break;
        case 0b1110:
            // A6-200
            break;
        }
    }
    else if (BITMATCH(opcode, 1, 0,   x, 1, x, x,  x, x, x,   0))
    {
        // data processing (plain binary immediate) A5-17
        uint8_t op = (arm >> 20) & 0x1f;
        uint8_t rn = (arm >> 16) & 0xf;
        switch (op)
        {
        case 0b00000:
            if (rn != 0xF)
            {
                // A6-22 ADD (imm) 12-bit
            }
            else
            {
                // A6-30 ADR
            }
            break;
        case 0b00100:
            // A6-147 MOV (imm) 16-bit
            break;
        case 0b01010:
            if (rn != 0xF)
            {
                // A6-244 SUB (imm) 12-bit
            }
            else
            {
                // A6-30 ADR
            }
            break;
        case 0b01100:
            // A6-153 MOVT (16-bit)
            break;
        case 0b10100:
            // A6-266 SBFX
            break;
        case 0b10110:
            if (rn != 0xF)
            {
                // A6-43 BFI
            }
            else
            {
                // A6-42 BFC
            }
            break;
        case 0b11100:
            // A6-266 UBFX
            break;
        }
    }
    else if (BITMATCH(opcode, 1, 0,   x, x, x, x,  x, x, x,   1))
    {
        uint32_t op2 = bits(arm, 12, 3);
        uint32_t op1 = bits(arm, 20, 7);
        if (BITMATCH(op2, 0, x, 0) && !BITMATCH(op1, x, 1, 1, 1, x, x, x))
        {
            // A6-40 conditional branch
            return 0;
        }
        else if (BITMATCH(op2, 0, x, 1))
        {
            // A6-40 branch
        }
        else if (BITMATCH(op2, 1, x, 1))
        {
            // A6-49 branch with link
            uint32_t sign = bit(arm, 26);
            uint32_t i1 = !(bit(arm, 13) ^ sign);
            uint32_t i2 = !(bit(arm, 11) ^ sign);
            uint32_t imm23 = (bits(arm, 0, 11) | (bits(arm, 16, 10) << 11) | (i1 << 21) | (i2 << 22)) << 1;
            uint32_t signextended = sign ? 0xff000000 : 0;
            int32_t offset = (int32_t)(imm23 | signextended);
            
            uintptr_t absaddr = offset + base;
            
            snprintf(chbuf, MAXLN, "BL %zx", absaddr);
            return 1;
        }
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, 0, x,  x, x, 0,   x))
    {
        // A5-25 store single data item
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, x, x,  0, 0, 1,   x))
    {
        // A5-24 load byte, mem hint
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, x, x,  0, 1, 1,   x))
    {
        // A5-23 load halfword, unalloc mem hint
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, 0, x,  1, 0, 1,   x))
    {
        // A5-22 load word
        uint32_t op1 = bit(arm, 23);
        uint32_t op2 = bits(arm, 6, 6);
        uint8_t rn = bits(arm, 16, 4);
        if (rn == 0xF)
        {
            // A6-90 LDR (literal)
        }
        else if (op1 || BITMATCH(op2, 1, x, x, 1, x, x) || BITMATCH(op2, 1, 1, 0, 0, x, x))
        {
            // A6-88 LDR (immediate)
            uint8_t rn = bits(arm, 16, 4);
            uint8_t p = bit(arm, 10);
            uint8_t u = bit(arm, 9);
            uint8_t w = bit(arm, 8);
            uint8_t imm8 = bits(arm, 0, 8);
            if (rn == 0b1101 && p == 0 && u == 1 && w == 1 && imm8 == 4)
            {
                // A6-186 POP
                uint8_t rt = bits(arm, 12, 4);
                if (rt == 13) return 0;
                
                snprintf(chbuf, MAXLN, "POP { %s }", rname(rt));
                return 1;
            }
        }
        else if (BITMATCH(op2, 1, 1, 1, 0, x, x))
        {
            // A6-133 LDRT
        }
        else if (op2 == 0)
        {
            // A6-92 LDR (register)
        }
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 0, x,  x, x, x,   x))
    {
        // A5-28 data processing (reg)
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 1, 0,  x, x, x,   x))
    {
        // A5-30 multiply / mult acc
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 1, 1,  x, x, x,   x))
    {
        // A5-31 long multiply, divide (reg)
    }
    else if (BITMATCH(opcode, 1, 1,   1, x, x, x,  x, x, x,   x))
    {
        // A5-32 coprocessor
    }
    
    return 0;
}

static int decode_16_datap(const uint16_t arm, uintptr_t base)
{
    assert(arm >> 10 == 0b010000);
    const uint16_t opcode = (arm >> 6) & 0xF;
    
    // except where otherwise noted, these are all single-register instructions
    switch (opcode)
    {
    case 0x0: // A6-34 AND (reg)
        return decode_binop_16_rm_rdn("AND", arm);
    case 0x1: // A6-74 EOR
        return decode_binop_16_rm_rdn("EOR", arm);
    case 0x2: // A6-136 LSL
        break;
    case 0x3: // A6-140 LSR
        break;
    case 0x4: // A6-38 ASR
        break;
    case 0x5: // A6-20 ADC
        break;
    case 0x6: // A6-206 SBC
        break;
    case 0x7: // A6-196 ROR
        break;
    case 0x8: // A6-264 TST
        break;
    case 0x9: // A6-200 RSB (imm)
        break;
    case 0xA: // A6-64 CMP
        break;
    case 0xB: // A6-60 CMN
        break;
    case 0xC: // A6-174 ORR (reg)
        return decode_binop_16_rm_rdn("ORR", arm);
    case 0xD: // A6-160 MUL (2x reg)
        break;
    case 0xE: // A6-64 BIC
        break;
    case 0xF: // A6-164 MVN 
        break;
    }
    return 0;
}

static int decode_16_arith(const uint16_t arm, uintptr_t base)
{
    assert(arm >> 14 == 0);
    const uint16_t opcode = (arm >> 9) & 0b0011111;
    
    switch (opcode)
    {
    case 0b00000:
    case 0b00001:
    case 0b00010:
    case 0b00011:
        // A6-134 lsl
        return decode_imm_shift(0, IMM5_RD_RM(arm));
    case 0b00100:
    case 0b00101:
    case 0b00110:
    case 0b00111:
        // A6-138 lsr
        return decode_imm_shift(1, IMM5_RD_RM(arm));
    case 0b01000:
    case 0b01001:
    case 0b01010:
    case 0b01011:
        // A6-36 asr
        return decode_imm_shift(0, IMM5_RD_RM(arm));
    case 0b01100:
        // A6-24 add
        {
        
        }
        break;
    case 0b01101:
        // A6-246 sub
        {
        }
        break;
    case 0b01110:
        // A6-22 add 3-bit imm
        {
        }
        break;
    case 0b01111:
        // A6-244 subtract 3-bit imm
        {
        }
        break;
    case 0b10000:
    case 0b10001:
    case 0b10010:
    case 0b10011:
        // A6-148 mov
        {
        }
        break;
    case 0b10100:
    case 0b10101:
    case 0b10110:
    case 0b10111:
        // A6-62 cmp
        {
            
        }
        break;
    case 0b11000:
    case 0b11001:
    case 0b11010:
    case 0b11011:
        // A6-22 add 8-bit imm
        {
        }
        break;
    case 0b11100:
    case 0b11101:
    case 0b11110:
    case 0b11111:
        // A6-244 sub 8-bit imm
        {
        }
        break;
    }
    
    return 0;
}

static int decode_16_misc(const uint16_t arm, uintptr_t base)
{
    // A5-10
    assert(arm >> 12 == 0xB);
    const uint16_t opcode = (arm >> 5) & 0b1111111;
    
    if (BITMATCH(opcode, 0, 0, 0, 1, x, x, x))
    {
        // A6-52 Compare and Branch on Zero
        return 0;
    }
    else if (BITMATCH(opcode, 0, 0, 1, 1, x, x, x))
    {
        // A6-52 Compare and Branch on Zero
        return 0;
    }
    else if (BITMATCH(opcode, 1, 0, 0, 1, x, x, x))
    {
        // A6-52 Compare and Branch on Non-Zero
        return 0;
    }
    else if (BITMATCH(opcode, 0, 1, 0, x, x, x, x))
    {
        // push
        uint8_t regs = arm & 0x00ff;
        uint8_t pushlr = (arm & 0x0100) >> 8;
        if (regs == 0 && pushlr == 0)
        {
            return 0;
        }
        snprintf(chbuf, MAXLN, "PUSH { %s%s%s%s%s%s%s%s%s }",
            OPREGSEP8,
            OPREGSEP(14, pushlr)
        );
        return 1;
    }
    else if (BITMATCH(opcode, 1, 1, 0, x, x, x, x))
    {
        // A6-186 pop
        uint8_t regs = arm & 0x00ff;
        uint8_t poppc = (arm & 0x0100) >> 8;
        if (regs == 0 && poppc == 0)
        {
            return 0;
        }
        snprintf(chbuf, MAXLN, "POP { %s%s%s%s%s%s%s%s%s }",
            OPREGSEP8,
            OPREGSEP(15, poppc)
        );
        return 1;
    }
    else if (BITMATCH(opcode, 1, 1, 1, 0, x, x, x))
    {
        return 0; // A6-48 breakpoint
    }
    else if (BITMATCH(opcode, 1, 1, 1, 1, x, x, x))
    {
        return 0; // A5-11 IT
    }
    
    switch(opcode)
    {
        // TODO
    }
    
    return 0;
}

static int decode_bx_16(const uint16_t arm, uintptr_t base)
{
    assert(arm >> 8 == 0b01000111);
    const int link = !! (arm & 0x0080);
    uint8_t reg = (arm >> 3) & 0x0f;
    if (reg == 15) return 0;
    snprintf(chbuf, MAXLN, "%s %s", link ? "BLX" : "BX", rname(reg));
    return 1;
}

static int decode_16(const uint16_t arm, uintptr_t base)
{
    const uint16_t opcode = (arm >> 10);
    if (BITMATCH(opcode, 0, 0, x, x, x, x))
    {
        return decode_16_arith(arm, base);
    }
    else if (
        BITMATCH(opcode, 0, 1, 0, 1, x, x)
        || BITMATCH(opcode, 0, 1, 1, x, x, x)
        || BITMATCH(opcode, 1, 0, 0, x, x, x)
    )
    {
        // A5-9 load single data item
        return 0;
    }
    
    // A5-2 16-bit Thumb instruction encoding
    
    switch(opcode)
    {
        case 0b010000:
            // A5-7 data processing
            return decode_16_datap(arm, base);
        case 0b010001:
            // A5-4 special data instructions
            switch ((arm >> 6) & 0x0f)
            {
            case 0b0000:
            case 0b0001:
            case 0b0010:
            case 0b0011:
                // A6-24 ADD (reg)
                break;
            case 0b0101:
            case 0b0110:
            case 0b0111:
                // A6-64 CMP (reg)
                break;
            case 0b1000:
            case 0b1001:
            case 0b1010:
            case 0b1011:
                // A6-150 MOV (reg)
                break;
            case 0b1100:
            case 0b1101:
                // A6-51 BX
            case 0b1110:
            case 0b1111:
                // A6-50 BLX
                return decode_bx_16(arm, base);
            }
            break;
        case 0b010010:
        case 0b010011:
            // A6-90 load from literal pool
            break;
        case 0b101000:
        case 0b101001:
            // A6-30 PC-relative address
            break;
        case 0b101010:
        case 0b101011:
            // A6-26 SP-relative address
            break;
        case 0b101100:
        case 0b101101:
        case 0b101110:
        case 0b101111:
            // misc, A5-10
            return decode_16_misc(arm, base);
        case 0b110000:
        case 0b110001:
            // A6-218 store multiple registers
            break;
        case 0b110010:
        case 0b110011:
            // A6-84 load multiple registers
            break;
        case 0b110100:
        case 0b110101:
        case 0b110110:
        case 0b110111:
            switch ((arm >> 8) & 0xF)
            {
            case 0xF:
                // A6-252 SVC/SWI supervisor call
                break;
            case 0xE:
                // permanently undefined
                break;
            default:
                // A6-40 conditional branch
                break;
            }
            break;
        case 0b111000:
        case 0b111001:
            // A6-40 unconditional branch
            break;
    }
    
    return 0;
}