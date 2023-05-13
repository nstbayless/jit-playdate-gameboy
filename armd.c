#include "armd.h"
#include "map.h"

#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

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
    
#ifdef TARGET_PLAYDATE
    #define PRIxPTR "%lx"
#else
    #define PRIxPTR "%x"
#endif

#ifdef TARGET_PLAYDATE
    #define PRIx32 "%lx"
#else
    #define PRIx32 "%x"
#endif

#define ptr_to_uint(x) ((uintptr_t)(void*)x)
#define TODO " [TODO]"

static int prop(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(chbuf, MAXLN, fmt, args);
    va_end(args);
    return 1;
}

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

#define REGS16 "{ %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s }"
#define REGS9 "{ %s%s%s%s%s%s%s%s%s }"
#define REGS8 "{ %s%s%s%s%s%s%s%s }"
#define OPREGSEP(i, v) ((v) ? regnames_comma[i] : "")
#define OPREGSEPM(i) OPREGSEP(i, (regs >> i) & 1)
#define OPREGSEP8 MAP_LIST(OPREGSEPM, 0, 1, 2, 3, 4, 5, 6, 7)
#define OPREGSEP16 MAP_LIST(OPREGSEPM, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15)

#define IMM5_RD_RM(u) bits(u, 6, 5), bits(u, 0, 3), bits(u, 3, 4)

const char* tmpxint(uint32_t i)
{
    char* buff = getbuff();
    snprintf(buff, IMMAX, "#$" PRIx32, i);
    return buff;
}

const char* expand_imm12(uint16_t imm12, int c)
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
        unsigned int unrot = bits(imm12, 0, 7) | 0x80;
        unsigned int rot = bits(imm12, 7, 5);
        
        uint32_t result = (unrot >> rot) | (unrot << (32-rot));
        int carry = !! (result & 0x80000000);
        if (rot == 0)
        {
            snprintf(buff, IMMAX, "#$??");
        }
        else if (c)
        {
            snprintf(buff, IMMAX, "#$" PRIx32 "; C <- %d", result, carry);
        }
        else
        {
            snprintf(buff, IMMAX, "#$" PRIx32, result);
        }
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
    return prop("%sS %s, %s", name, rname(rdn), rname(rm));
}

static uint16_t imm12_0_12_26(uint32_t arm)
{
    return bits(arm, 0, 8) | (bits(arm, 12, 3) << 8) | (bit(arm, 26) << 11);
}

static uint16_t imm16_0_12_26_16(uint32_t arm)
{
    return bits(arm, 0, 8) | (bits(arm, 12, 3) << 8) | (bit(arm, 26) << 11) | (bits(arm, 16, 4) << 12);
}


#define RNOFF1 1
#define EXPAND12C 6
#define EXPAND12 4

static int decode_op32_S20_rd8_imm12_0_12_10(const char* name, uint32_t arm, int flags)
{
    uint8_t rd = bits(arm, 8, 4);
    uint8_t setflags = bit(arm, 20);
    if (rd == 13 || rd == 15) return 0;
    uint16_t imm12 = imm12_0_12_26(arm);
    const char* imm = NULL;
    if (flags & EXPAND12)
    {
        imm = expand_imm12(imm12, flags & EXPAND12C);
    }
    else
    {
        imm = tmpxint(imm12);
    }
    if (imm == NULL) return 0;
    return prop("%s%s %s,%s", name, SSFLAG(setflags), rname(rd), imm);
}

static int decode_op32_s20_rn8_rd16_imm12_0_12_26(const char* name, uint32_t arm, int flags)
{
    int rnoff = !!(flags & RNOFF1);
    uint8_t rd = bits(arm, 8, 4);
    uint8_t rn = bits(arm, 16+rnoff, 4);
    uint16_t imm12 = imm12_0_12_26(arm);
    uint8_t setflags = bit(arm, 20);
    if (rd == 13 || rd == 15 || rn == 13) return 0;
    
    const char* imm = NULL;
    if (flags & EXPAND12)
    {
        imm = expand_imm12(imm12, setflags && (flags & EXPAND12C));
    }
    else
    {
        imm = tmpxint(imm12);
    }
    if (imm == NULL) return 0;
    return prop("%s%s %s,%s,%s", name, SSFLAG(setflags), rname(rd), rname(rn), imm);
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
    
    prop("%s%s.W %s, %s, %s", name, SSFLAG(setflags), rname(rd), rname(rn), regshift(rm, type, imm5));
    return 1;
}

static int decode_imm_shift(uint8_t shift_type, uint8_t imm5, uint8_t rd, uint8_t rm)
{
    switch(shift_type & 3)
    {
    case 0:
        return prop("LSL %s, %s, %d", rname(rd), rname(rm), imm5);
    case 1:
        return prop("LSR %s, %s, %d", rname(rd), rname(rm), imm5 == 0 ? 32 : imm5);
    case 2:
        return prop("ASR %s, %s, %d", rname(rd), rname(rm), imm5 == 0 ? 32 : imm5);
    case 3:
        if (imm5 == 0)
            return prop("RRX %s, %s, 1", rname(rd), rname(rm));
        else
            return prop("ROR %s, %s, %d", rname(rd), rname(rm), imm5);
    }
    
    return 0;
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
        int is_alt_behaviour = (bits(arm, 16, 4) | (bit(arm, 21) << 4)) == 0b11101;
        
        uint16_t regs = bits(arm, 0, 16);
        uint8_t w = bit(arm, 21);
        uint8_t rn = bits(arm, 16, 4);
        
        switch(op)
        {
        case 0b010:
            // A6-218 STM T2 -- store multiple
            if (bit(regs, 13) || bit(regs, 15)) return 0;
            if (__builtin_popcount(regs) <= 1) return 0;
            if (rn == 15) return 0;
            return prop("STM.W %s%s " REGS16, rname(rn), w ? "!" : "", OPREGSEP16);
        case 0b011:
            if (!is_alt_behaviour)
            {
                // A6-84 LDM -- load multiple
                if (rn == 15 || __builtin_popcount(regs) <= 1) return 0;
                if (bit(regs, 14) && bit(regs, 15)) return 0;
                if (w && bit(regs, rn)) return 0;
                return prop("LDM.W %s%s " REGS16, rname(rn), w ? "!" : "", OPREGSEP16);
            }
            else
            {
                // A6-186 pop
                if (bit(regs, 14) && bit(regs, 15)) return 0;
                if (__builtin_popcount(regs) <= 1) return 0;
                
                return prop("POP " REGS16,
                    OPREGSEP16
                );
            }
            break;
        case 0b100:
            if (!is_alt_behaviour)
            {
                // A6-220 STMDB -- store multiple decrement before
                if (bit(regs, 13) || bit(regs, 15)) return 0;
                if (rn == 15 || __builtin_popcount(regs) <= 1) return 0;
                if (w && bit(regs, rn)) return 0;
                return prop("STMDB %s%s " REGS16, rname(rn), w ? "!" : "", OPREGSEP16);
            }
            else
            {
                // A6-188 push T2
                if (bit(regs, 13) || bit(regs, 15)) return 0;
                if (__builtin_popcount(regs) <= 1) return 0;
                
                return prop("PUSH.W " REGS16,
                    OPREGSEP16
                );
            }
            break;
        case 0b101:
            // A6-286 load multiple
            return prop(TODO "[A6-286]");
            break;
        }
    }
    else if (BITMATCH(opcode, 0, 1,   0, 0, x, x,  1, x, x,   x))
    {
        // A5-21 load/store dual/exclusive
        return prop(TODO "[A5-21]");
    }
    else if (BITMATCH(opcode, 0, 1,   0, 1, x, x,  x, x, x,   x))
    {
        // A5-26 data processing (shifted register)
        // all operands are (reg)
        int s = bit(arm, 20);
        uint8_t op = bits(arm, 21, 4);
        uint8_t rd = bits(arm, 8, 4);
        uint8_t rn = bits(arm, 16, 4);
        
        switch(op)
        {
        case 0b0000:
            if (s == 1)
            {
                // A6-264 TST
                return prop(TODO "[A6-264]");
            }
            else if (rd != 0xF)
            {
                // A6-34 AND (reg)
                return decode_op32_rd_rn_rmshift("AND", arm);
            }
            else
            {
                return prop("ERR [A5-26]");
            }
        case 0b0001:
            // A6-46 BIC
            return prop(TODO "[A6-46]");
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
                uint32_t imm5 = (bits(arm, 12, 3) | (bits(arm, 6, 2) << 3));
                uint32_t type = bits(arm, 4, 2);
                switch(type)
                {
                case 0b00:
                    if (imm5 == 0)
                    {
                        // A6-150 MOV (reg) T3
                        uint8_t rd = bits(arm, 8, 4);
                        uint8_t rm = bits(arm, 0, 4);
                        uint8_t setflags = bit(arm, 20);
                        return prop("MOV%s", setflags ? "S" : "", rname(rd), rname(rm));
                    }
                    else
                    {
                        // A6-134 LSL (imm)
                        return prop("LSL" TODO);
                    }
                case 0b01:
                    // A6-138 LSR
                    return prop("LSR" TODO);
                case 0b10:
                    // A6-36 ASR
                    return prop("ASR" TODO);
                case 0b11:
                    if (imm5 == 0)
                    {
                        // A6-198 RRX
                        return prop("RRX" TODO);
                    }
                    else
                    {
                        // A6-194 ROR
                        return prop("ROR" TODO);
                    }
                }
            }
            break;
        case 0b0011:
            if (rn != 0xF)
            {
                // A6-170 ORN
                return prop(TODO "[A6-170]");
            }
            else
            {
                // A6-164 MVN
                return prop(TODO "[A6-164]");
            }
        case 0b0100:
            if (rd != 0xF)
            {
                // A6-74 EOR (reg)
                return decode_op32_rd_rn_rmshift("EOR", arm);
            }
            else if (s == 1)
            {
                // A6-261 TEQ
                return prop(TODO "[A6-261]");
            }
        case 0b1000:
            if (rd != 0xF)
            {
                // A6-24 ADD
                return prop(TODO "[A6-24]");
            }
            else if (s == 1)
            {
                // A6-60 CMN
                return prop(TODO "[A6-60]");
            }
        case 0b1010:
            // A6-20 ADC
            return prop(TODO "[A6-20]");
        case 0b1011:
            // A6-206 SBC
            return prop(TODO "[A6-206]");
        }
        return prop(TODO "[A5-26]");
    }
    else if (BITMATCH(opcode, 0, 1,   1, x, x, x,  x, x, x,   x))
    {
        // A5-32 coprocessor
        return prop(TODO "[A5-32 coprocessor]");
    }
    else if (BITMATCH(opcode, 1, 0,   x, 0, x, x,  x, x, x,   0))
    {
        // data processing (modified immediate) A5-14
        // all operands are (imm).
        uint8_t op = bits(arm, 21, 4);
        uint8_t rd = bits(arm, 8, 4);
        uint8_t rn = bits (arm, 16, 4);
        
        switch (op)
        {
        case 0b0000:
            if (rd != 0xF)
            {
                // A6-32 AND (imm)
                return decode_op32_s20_rn8_rd16_imm12_0_12_26("AND", arm, EXPAND12C);
            }
            else
            {
                // A6-262 TST
                return prop("TST" TODO);
            }
        case 0b0001:
            return prop("BIC" TODO);
        case 0b0010:
            if (rn != 0xF)
            {
                // A6-172 ORR (imm)
                return decode_op32_s20_rn8_rd16_imm12_0_12_26("ORR", arm, EXPAND12C);
            }
            else
            {
                // A6-148 MOV (imm) T2
                return decode_op32_S20_rd8_imm12_0_12_10("MOV", arm, EXPAND12C);
            }
        case 0b0011:
            if (rn != 0xF)
            {
                // A6-168 ORN (imm)
                return decode_op32_s20_rn8_rd16_imm12_0_12_26("ORN", arm, EXPAND12C);
            }
            else
            {
                // A6-162 MVN
                return prop("MVN" TODO);
            }
        case 0b0100:
            if (rd != 0xF)
            {
                // A6-72 EOR
                return decode_op32_s20_rn8_rd16_imm12_0_12_26("EOR", arm, EXPAND12C | RNOFF1);
            }
            else
            {
                // A6-260 TEQ
                return prop("TEQ" TODO);
            }
        case 0b1000:
            if (rd != 0xF)
            {
                // A6-22 ADD (imm) T3
                decode_op32_s20_rn8_rd16_imm12_0_12_26("ADD", arm, EXPAND12);
            }
            else
            {
                // A6-58 CMN
                return prop("CMN" TODO);
            }
        case 0b1010:
            // A6-18 ADC
            return prop("ADC" TODO);
        case 0b1011:
            // A6-204 SBC
            return prop("SBC" TODO);
        case 0b1101:
            if (rd != 0xF)
            {
                // A6-244 SUB
                return prop("SUB" TODO);
            }
            else
            {
                // A6-62 CMP
                return prop("CMP" TODO);
            }
            break;
        case 0b1110:
            // A6-200 RSB
            return prop("RSB" TODO);
        }
    }
    else if (BITMATCH(opcode, 1, 0,   x, 1, x, x,  x, x, x,   0))
    {
        // data processing (plain binary immediate) A5-17
        uint8_t op = bits(arm, 20, 5);
        uint8_t rn = bits(arm, 16, 4);
        switch (op)
        {
        case 0b00000:
            if (rn != 0xF)
            {
                // A6-22 ADD (imm) T4
                return decode_op32_s20_rn8_rd16_imm12_0_12_26("ADDW", arm, 0);
            }
            else
            {
                // A6-30 ADR
                return prop("ADR" TODO);
            }
            break;
        case 0b00100:
            // A6-148 MOV (imm) T3 16-bit
            {
                uint32_t imm16 = imm16_0_12_26_16(arm);
                uint8_t rd = bits(arm, 8, 4);
                return prop("MOVW %s,#$%04x", rname(rd), imm16);
            }
        case 0b01010:
            if (rn != 0xF)
            {
                // A6-244 SUB (imm) 12-bit
                return prop("SUB" TODO);
            }
            else
            {
                // A6-30 ADR
                return prop("ADR" TODO);
            }
        case 0b01100:
            // A6-153 MOVT (16-bit)
            {
                uint32_t imm16 = imm16_0_12_26_16(arm);
                uint8_t rd = bits(arm, 8, 4);
                return prop("MOVT %s,#$%04x", rname(rd), imm16);
            }
        case 0b10100:
            // A6-266 SBFX
            return prop("SBFX" TODO);
        case 0b10110:
            {
                uint8_t msb = bits(arm, 0, 5);
                uint8_t lsb = bits(arm, 6, 2) | (bits(arm, 12, 3) << 2);
                uint8_t width = msb-lsb+1;
                uint8_t rd = bits(arm, 8, 4);
                if (rn != 0xF)
                {
                    // A6-43 BFI
                    return prop("BFI %s, %s, #%d, #%d", rname(rd), rname(rn), (int)lsb, (int)width);
                }
                else
                {
                    // A6-42 BFC
                    return prop("BFC %s, #%d, %d", rname(rd), (int)lsb, (int)width);
                }
            }
            break;
        case 0b11100:
            // A6-266 UBFX
            return prop("UBFX" TODO);
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
            return prop(TODO "[A6-40]");
        }
        else if (BITMATCH(op2, 0, x, 1))
        {
            // A6-40 branch
            return prop(TODO "[A6-40]");
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
            
            return prop("BL " PRIxPTR, absaddr);
        }
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, 0, x,  x, x, 0,   x))
    {
        // A5-25 store single data item
        return prop(TODO "[A5-25]");
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, x, x,  0, 0, 1,   x))
    {
        // A5-24 load byte, mem hint
        return prop(TODO "[A5-24]");
    }
    else if (BITMATCH(opcode, 1, 1,   0, 0, x, x,  0, 1, 1,   x))
    {
        // A5-23 load halfword, unalloc mem hint
        return prop(TODO "[A5-23]");
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
            return prop(TODO "[A6-90]");
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
                
                return prop("POP { %s }", rname(rt));
            }
        }
        else if (BITMATCH(op2, 1, 1, 1, 0, x, x))
        {
            // A6-133 LDRT
            return prop(TODO "[A6-133]");
        }
        else if (op2 == 0)
        {
            // A6-92 LDR (register)
            return prop(TODO "[A6-92]");
        }
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 0, x,  x, x, x,   x))
    {
        // A5-28 data processing (reg)
        return prop(TODO "[A5-28]");
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 1, 0,  x, x, x,   x))
    {
        // A5-30 multiply / mult acc
        return prop(TODO "[A5-30]");
    }
    else if (BITMATCH(opcode, 1, 1,   0, 1, 1, 1,  x, x, x,   x))
    {
        // A5-31 long multiply, divide (reg)
        return prop(TODO "[A5-31]");
    }
    else if (BITMATCH(opcode, 1, 1,   1, x, x, x,  x, x, x,   x))
    {
        // A5-32 coprocessor
        return prop(TODO "[A5-32]");
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
            uint8_t imm3 = bits(arm, 6, 3);
            uint8_t rn = bits(arm, 3, 3);
            uint8_t rd = bits(arm, 0, 3);
            return prop("ADDS %s,%s,%02x", rname(rd), rname(rn), imm3);
        }
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
            uint8_t rd = bits(arm, 8, 3);
            uint32_t imm8 = bits(arm, 0, 8);
            return prop("MOVS %s, #$%02x", rname(rd), imm8);
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
    const uint16_t opcode = bits(arm, 5, 7);
    
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
        // A6-188 push
        uint8_t regs = bits(arm, 0, 8);
        uint8_t pushlr = bit(arm, 8);
        if (regs == 0 && pushlr == 0)
        {
            return 0;
        }
        return prop("PUSH " REGS9,
            OPREGSEP8,
            OPREGSEP(14, pushlr)
        );
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
        return prop("POP " REGS9,
            OPREGSEP8,
            OPREGSEP(15, poppc)
        );
    }
    else if (BITMATCH(opcode, 1, 1, 1, 0, x, x, x))
    {
        return 0; // A6-48 breakpoint
    }
    else if (BITMATCH(opcode, 1, 1, 1, 1, x, x, x))
    {
        if (arm == 0xbf00) return prop("NOP");
        return 0; // A5-11 IT
    }
    
    switch(opcode)
    {
    case 0b0000000:
    case 0b0000001:
    case 0b0000010:
    case 0b0000011:
        // A6-26 ADD (SP plus immediate)
        break;
    case 0b0000100:
    case 0b0000101:
    case 0b0000110:
    case 0b0000111:
        // A6-248 SUB (SP minus immediate)
        break;
    case 0b0010000:
    case 0b0010001:
        {
            uint8_t rd = bits(arm, 0, 3);
            uint8_t rm = bits(arm, 3, 3);
            return prop("SXTH %s, %s", rname(rd), rname(rm));
        }
        break;
    case 0b0010010:
    case 0b0010011:
        {
            uint8_t rd = bits(arm, 0, 3);
            uint8_t rm = bits(arm, 3, 3);
            return prop("SXTB %s, %s", rname(rd), rname(rm));
        }
        break;
    case 0b0010100:
    case 0b0010101:
        {
            uint8_t rd = bits(arm, 0, 3);
            uint8_t rm = bits(arm, 3, 3);
            return prop("UXTH %s, %s", rname(rd), rname(rm));
        }
    case 0b0010110:
    case 0b0010111:
        {
            uint8_t rd = bits(arm, 0, 3);
            uint8_t rm = bits(arm, 3, 3);
            return prop("UXTB %s, %s", rname(rd), rname(rm));
        }
    case 0b1010000:
    case 0b1010001:
        // A6-191 REV
        break;
    case 0b1010010:
    case 0b1010011:
        // A6-191 REV16
        break;
    case 0b1010110:
    case 0b1010111:
        // A6-191 REVSH
        break;
    }
    
    return 0;
}

static int decode_bx_16(const uint16_t arm, uintptr_t base)
{
    assert(arm >> 8 == 0b01000111);
    const int link = !! (arm & 0x0080);
    uint8_t reg = (arm >> 3) & 0x0f;
    if (reg == 15) return 0;
    prop("%s %s", link ? "BLX" : "BX", rname(reg));
    return 1;
}

static int decode_16_single_data_item(const uint16_t arm, uintptr_t base)
{
    // A5-9
    uint8_t op = bits(arm, 9, 7);
    switch (op)
    {
    case 0b0101000:
        // A6-224 STR (reg)
        break;
    case 0b0101001:
        // A6-240 STRH (reg)
        break;
    case 0b0101010:
        // A6-228 STRB (reg)
        break;
    case 0b0101011:
        // A6-122 LDRSB (reg)
        break;
    case 0b0101100:
        // A6-92 LDR (reg)
        break;
    case 0b0101101:
        // A6-114 LDRH (reg)
        break;
    case 0b0101110:
        // A6-98 LDRB (reg)
        break;
    case 0b0101111:
        // A6-130 LDRSH (reg)
        break;
    }
    
    uint8_t rn = bits(arm, 3, 3);
    uint8_t rt = bits(arm, 0, 3);
    uint8_t imm5 = bits(arm, 6, 5);
    switch (bits(arm, 11, 5))
    {
    case 0b01100:
        // A6-222 STR (immediate)
        {
            return prop("STR %s, [%s,%02x]", rname(rt), rname(rn), imm5<<2);
        }
    case 0b01101:
        // A6-88 LDR (immediate)
        return prop("LDR %s, [%s,%02x]", rname(rt), rname(rn), imm5 << 2);
    case 0b01110:
        // A6-226 STRB (immediate)
        return prop("STRB %s, [%s,%02x]", rname(rt), rname(rn), imm5);
    case 0b01111:
        // A6-94 LDRB (immediate)
        return prop("LDRB %s, [%s,%02x]", rname(rt), rname(rn), imm5);
    case 0b10000:
        // A6-238 STRH (immediate)
        return prop("STRH %s, [%s,%02x]", rname(rt), rname(rn), imm5 << 1);
    case 0b10001:
        // A6-110 LDRH (immediate)
        return prop("LDRH %s, [%s,%02x]", rname(rt), rname(rn), imm5 << 1);
    case 0b10010:
        // A6-222 STR (immediate) T2
        return prop("STR %s, [SP,%02x]", rname(bits(arm, 8, 3)), bits(arm, 0, 8) << 2);
    case 0b10011:
        // A6-88 LDR (immediate) T2
        return prop("LDR %s, [SP,%02x]", rname(bits(arm, 8, 3)), bits(arm, 0, 8) << 2);
    }
    
    return 0;
}

static int decode_16(const uint16_t arm, uintptr_t base)
{
    uint8_t regs, rn;
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
        return decode_16_single_data_item(arm, base);
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
            regs = bits(arm, 0, 8);
            rn = bits(arm, 8, 3);
            return prop("STM %s, " REGS8, rname(rn), OPREGSEP8);
        case 0b110010:
        case 0b110011:
            // A6-84 load multiple registers
            regs = bits(arm, 0, 8);
            rn = bits(arm, 8, 3);
            return prop("LDM %s, " REGS8, rname(rn), OPREGSEP8);
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