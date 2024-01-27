#include "jit.h"
#include "armd.h"
#include "sm38d.h"
#include "map.h"

#ifdef __GNUC__
    #define __forceinline __attribute__((always_inline))
#elif !defined(_MSC_VER)
    #define __forceinline
#endif

/*
    Useful resources when developing this JIT:
    - Arm v7-m architecture reference manual: 
        - https://developer.arm.com/documentation/ddi0489/b/
        - This will help convert between arm opcodes and hexadecimal.
    - Godbolt compiler: https://godbolt.org/z/ornhbWjee
        - see what gcc thinks is the most efficient way to perform your desired operation.
    - ARM-to-hex:
        - https://armconverter.com/ (thumb output)
        - note that the byte-order is weird... it's like 2-3-0-1-ordered in significance.
    - Arm Cortex M-7 reference manual (could be useful for cycle counting and I/O registers?)
    - ARM assembler:
        - https://onecompiler.com/assembly/3z7z43ypn
*/

/*
    How this dynarec/JIT works:
        - converts a short sequence of sm83 (gameboy) machine code to a callable subroutine of thumb (arm cortex m-7) machine code.
            - we call this subroutine the "resultant chunk," or just chunk.
        - each sm83 instruction can be multiple ARM instructions.
        - only sm83 instructions from ROM are translated; if the gameboy's program counter is in ram or somewhere else exotic, the dynarec is skipped.
        - additionally, some arm instructions are added to the start and end ("prologue" and "epilogue") of the chunk to interface with emulator (loading/storing registers, etc.).
        - arm registers obey these invariants, which are assumed at the start and end of each transpiled sm83 instruction, as well as at the end of the prologue and the start of the epilogue:
            - r1 = A (high 3 bytes are always 0)
            - r2 = HL (high 2 bytes are always 0)
            - r3 = DE (high 2 bytes are always 0)
            - r4 = BC (high 2 bytes are always 0)
            - r5 = carry (only 1 bit is used)
            - r6 = !z flag (0 iff z is set)
            - r7 = n and h flags (see explanation in jit.h)
            - r8 = SP (high 2 bytes are always 0)
            - (r0 is used as flex/scratch, and to return cycle count at the end)
            - (note that some registers might not be loaded from jit_regfile_t in the prologue if they are not inputs for this chunk)
            - (similarly, some registers might not be written to jit_regfile_t in the epilogue if they are not modified this chunk)
        - dynamic recompiling is done by translating in the following passes:
            1. Fragmentation: each sm83 instruction is converted into multiple "translation fragments," or just "fragments" for short (frag_t).
                - a fragment (frag_t) is a representation of an operation in arm state.
                - for example, a fragment could represent a single 16-bit thumb instruction, or it could be "call a function at a particular address" (a multi-instruction operation)
                - some fragments are added at the start and end as prologue/epilogue.
                - during this step, we keep track of what arm registers are "used," "input," and "dirty" (aka "output").
                    - "used": the arm register was either 
                    - "input": the arm register is read before it is written to; it must be loaded in the prologue.
                    - "dirty": the arm register is modified and needs to be stored to jit_regfile_t in the epilogue.
                        - we only care about storing dirty registers if they correspond to a sm83 register
                        - the sm83 pc is assumed to always end up dirty, so we just store it in the epilogue (statically calculated).
            2. Length: each fragment is then polled to get (an upper bound of) how long it is in terms of half-words (16-bits, i.e. thumb instruction alignment).
            3. Assembly: each fragment is then assembled to a sequence of thumb instructions into a buffer of length equaling the sum of the fragment lengths.
                - This buffer can now be called like any C function. It returns (in r0) the number of cycles that have passed.
        - We can summarize the transformation of data then as: byte sm83[] -> frag_t fragments[] (including prologue/epilogue) -> byte thumb[].
        - the cycle count is estimated by summing the cycle count for each fragment (at time of dynamic recompilation), and returning this value in r0.
        - not implemented yet:
            - r0 cycle count return
            - storing pc.
            - branching.
            - status flags
*/

/*
    OPTIMIZATION IDEAS
    - we can combine multiple rotate instructions in a row
    - these sm83 commands don't exist: ld bc,hl; ld de,bc; and so on.
        - they are emulated in sm83 by instruction pairs e.g. ld b,h; ld c, l
        - we can identify this and reduce to just one arm mov instruction.
        
    - we can analyze the rst routines and inline them
    
    - store status flags in r2, r3;
      that way we can clobber them freely during function calls
      most of the time.
      
    -if writeword takes a uint_32 instead of uint16_t, we can avoid the uxth
    
    - sm83 rotate instructions can be improved a lot by leveraging the arm bfi instruction.
*/

#ifndef JIT_DEBUG
    //#define JIT_DEBUG
#endif

#define SETFLAGS 1

#ifdef TARGET_PLAYDATE
    #define assert(_B) jit_assert_pd(_B, opts.playdate)
#else
    #include <assert.h>
#endif

#ifdef TARGET_QEMU
    #include <stdio.h>
    #ifdef assert
        #undef assert
    #endif
    
    #define assert(statement) \
        if (statement) {} else { \
            printf("ASSERTION FAILED %s:%d: " #statement "\n", __FILE__, __LINE__); \
            exit(1);\
        }
    
#endif

#ifndef false
    #define false 0
#endif
#ifndef true
    #define true 1
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#define JIT_HASHTABLE_SIZE 0x4000 // must be power of 2, at most 0x8000
#define JIT_MAX_LABELS 8
#define JIT_START_PADDING_ENTRY_C 10

#ifdef JIT_DEBUG
    #ifdef TARGET_QEMU
        extern void _printf(const char* fmt, ...);
        #define JIT_DEBUG_MESSAGE(fmt, ...) _printf(fmt "\n", ##__VA_ARGS__)
    #else
        #define JIT_DEBUG_MESSAGE(...) do {opts.playdate->system->logToConsole(__VA_ARGS__); spin();} while(0)
    #endif
#else
    #define JIT_DEBUG_MESSAGE(...)
#endif

#ifdef ADC
    #undef ADC
#endif

#ifdef IN
    #undef IN
#endif

#ifdef OUT
    #undef OUT
#endif

// TODO: assign registers programatically
// reg_flex MUST be reg 0, because r0 is the return register.
#define REG_FLEX 0

// keep in mind that 4+ are caller-saved
// these are the arm registers used to store the given sm83 registers
// the are in the same order as the fields in jit_regfile_t to improve the behaviour of
// stm/ldm instructions.
#define REG_IDX(FIELD) (1 + offsetof(jit_regfile_t, FIELD) / sizeof(uint32_t))

#define REG_A REG_IDX(a)
#define REG_BC REG_IDX(bc)
#define REG_DE REG_IDX(de)
#define REG_HL REG_IDX(hl)
#define REG_SP REG_IDX(sp)
#define REG_Carry REG_IDX(carry)
#define REG_nZ REG_IDX(z) /* complement of sm83 z flag; is 0 iff sm83 z is set. */
#define REG_NH REG_IDX(nh) /* N and H flags; see jit.h */

// [sic], same opcode, but lsl has non-zero immediate
#define THUMB16_MOV_REG 0x0000
#define THUMB16_ADD_REG_T1 0x1800 /* rd0_rn3_rm6 */
#define THUMB16_SUB_REG 0x1A00 /* rd0_rn3_rm6 */
#define THUMB16_MOV_REG_T2 0x4600 /* rdn0127_rm_3456 */
#define THUMB16_ADD_3   0x1C00
#define THUMB16_SUB_3   0x1E00
#define THUMB16_LSL_IMM 0x0000
#define THUMB16_LSR_IMM 0x0800
#define THUMB16_ASR_IMM 0x1000
#define THUMB16_ADD_8   0x3000
#define THUMB16_SUB_8   0x3800
#define THUMB16_AND_REG 0x4000
#define THUMB16_EOR_REG 0x4040
#define THUMB16_ORR_REG 0x4300
#define THUMB16_ADD_REG_T2 0x4400 /* rdn0127_rm_3456 */
#define THUMB16_UXTH    0xB280
#define THUMB16_UXTB    0xB2C0
#define THUMB16_STR     0x6000
#define THUMB16_LDR     0x6800 /* actually this is LDR_IMM_T1 */
#define THUMB16_LDR_IMM_T2     0x9800
#define THUMB16_STRH    0x8000
#define THUMB16_CBZ     0xB100
#define THUMB16_CBNZ    0xB900
#define THUMB16_STM     0xC000
#define THUMB16_LDM     0xC800

#define THUMB32_STM     0xe8800000
#define THUMB32_LDM     0xe8900000
#define THUMB32_AND_REG 0xea000000
#define THUMB32_ORR_REG 0xea400000
#define THUMB32_EOR_REG 0xea800000
#define THUMB32_ADD_REG 0xeb000000
#define THUMB32_SUB_REG 0xeba00000
#define THUMB32_AND_IMM 0xf0000000
#define THUMB32_BIC_IMM 0xf0200000
#define THUMB32_ORR_IMM 0xf0400000
#define THUMB32_ORN_IMM 0xf0600000
#define THUMB32_EOR_IMM 0xf0800000
#define THUMB32_ADD_IMM_T3 0xf1000000
#define THUMB32_ADD_IMM_T4 0xf2000000
#define THUMB32_SUB_IMM_T3 0xf1A00000
#define THUMB32_SUB_IMM_T4 0xf2A00000
#define THUMB32_BFI     0xf3600000
#define THUMB32_SXTH    0xfa0ff080
#define THUMB32_UXTH    0xfa1ff080
#define THUMB32_SXTB    0xfa4ff080
#define THUMB32_UXTB    0xfa5ff080

// all sm83 registers
#define REGS_SM83 ((1 << REG_A) | (1 << REG_BC) | (1 << REG_DE) | (1 << REG_HL) | (1 << REG_SP) | (1 << REG_Carry) | (1 << REG_nZ) | (1 << REG_NH))
#define REGS_ALL (REGS_SM83 | 1)
#define REGS_NONFLEX (REGS_ALL & ~1)

#define offsetof_hi(a, b) (offsetof(a, b) + 1)

typedef void (*fn_type)(void);

typedef struct
{
    uint16_t addr;
    uint16_t bank;
    jit_fn fn;
} jit_ht_entry;

// all possible sm83 instruction operands and condition codes
// suffix: m = memory (dereference)
//         mi = dereference increment
//         md = dereference decrement
//         sp = plus stack pointer
typedef enum
{
    OPARG_A, OPARG_B, OPARG_C, OPARG_D, OPARG_E, OPARG_H, OPARG_L,
    OPARG_AF, OPARG_BC, OPARG_DE, OPARG_HL, OPARG_SP,
    OPARG_Cm, // appears in opcodes 0xE2 and 0xF2 only
    OPARG_BCm, OPARG_DEm, OPARG_HLm, OPARG_HLmi, OPARG_HLmd,
    OPARG_i8, OPARG_i16,
    OPARG_i8sp, // appears in opcode 0xF8 only
    OPARG_i8m, OPARG_i16m,
    
    // condition codes
    COND_none, COND_z, COND_c, COND_nz, COND_nc,
    
    // jump destinations
    ADDR_r8, ADDR_d16, ADDR_HL // relative 8; direct 16; indirect HL
} sm83_oparg_t;

static jit_opts opts;
static jit_ht_entry* jit_hashtable[JIT_HASHTABLE_SIZE];
static jit_ht_entry jit_default_entry = {
    .addr = 0,
    .bank = 0,
    .fn = NULL
};

static void* arm_interworking_none(void* addr)
{
	#ifdef __arm__
	return (void*)((uintptr_t)addr & ~1);
	#else
	return addr;
	#endif
}

static void* arm_interworking_thumb(void* fn)
{
    #ifdef __arm__
    return (void*)((uintptr_t)fn | 1);
    #else
    return fn;
    #endif
}

static uint32_t bits(uint32_t src, uint32_t lowbit, uint32_t bitc)
{
    return (src >> lowbit) & ((1 << bitc) - 1);
}

static uint32_t bit(uint32_t src, uint32_t bit)
{
    return bits(src, bit, 1);
}

static void jit_add_ht_entry(uint16_t gb_addr, uint16_t gb_bank, jit_fn fn)
{
    uint16_t hash_entry = (gb_addr) % JIT_HASHTABLE_SIZE;
    jit_ht_entry* prev = jit_hashtable[hash_entry];
    
    uint16_t prev_size = 0;
    for (const jit_ht_entry* p = prev; p != &jit_default_entry && p->fn; ++p)
    {
        prev_size++;
    }
    
    // add 2; 1 for new entry, 1 for terminal.
    jit_ht_entry* n = malloc(sizeof(jit_ht_entry) * (prev_size + 2));
    
    if (!n)
    {
        return;
    }
    
    memcpy(n, prev, sizeof(jit_ht_entry) * prev_size);
    
    // new entry
    n[prev_size].addr = gb_addr;
    n[prev_size].bank = gb_bank;
    n[prev_size].fn = fn;
    
    // terminal
    n[prev_size+1].fn = NULL;
    
    jit_hashtable[hash_entry] = n;
    if (prev != &jit_default_entry)
    {
        free(prev);
    }
}

// gets jit fn if it exists, otherwise returns NULL.
static inline __forceinline jit_fn get_jit_fn(uint16_t gb_addr, uint16_t gb_bank)
{
    uint16_t hash_entry = (gb_addr) % JIT_HASHTABLE_SIZE;
    jit_ht_entry* e = jit_hashtable[hash_entry];
    
    while (e->fn)
    {
        if (e->addr == gb_addr && e->bank == gb_bank) return e->fn;
        ++e;
    }
    
    return NULL;
}

#ifdef JIT_DEBUG
static void spin(void)
{
    for (size_t i = 0; i < 0x40000; ++i) asm("nop");
}
#endif

void jit_init(jit_opts _opts)
{
    opts = _opts;
    //JIT_DEBUG_MESSAGE("memset.\n");
    for (size_t i = 0; i < JIT_HASHTABLE_SIZE; ++i)
    {
        jit_hashtable[i] = &jit_default_entry;
    }
    //JIT_DEBUG_MESSAGE("memfix.\n");
    jit_memfix();
}

void jit_cleanup(void)
{
    for (size_t i = 0; i < JIT_HASHTABLE_SIZE; ++i)
    {
        if (jit_hashtable[i] != &jit_default_entry)
        {
            for (jit_ht_entry* e = jit_hashtable[i]; e->fn; ++e)
            {
                free(arm_interworking_none(e->fn));
            }
            free(jit_hashtable[i]);
        }
    }
    memset(&opts, 0, sizeof(opts));
}

void jit_memfix(void)
{
    #ifdef TARGET_PLAYDATE
    //memfix();
    #endif
}

/*
    Represents an arm instruction or short sequence of arm instructions.
    One sm83 instructions can map to multiple resultant fragments (frag_t)
*/
typedef struct {
    void* args;
    
    // if `outbuff` is NULL, don't actually write anything.
    // return size of (what would be) written
    uint8_t (*produce)(void* args, uint16_t* outbuff);
    
    #ifdef JIT_DEBUG
    const uint8_t* romsrc;
    uint8_t length; // this is set from produce() and recorded for debugging purposes
    #endif
} frag_t;

// translation state.
static struct {
    frag_t* frag;
    unsigned fragc;
    unsigned fragcap;
    const uint8_t* romstart;
    const uint8_t* rom;
    const uint8_t* romend;
    const uint8_t* initrom; // ptr to rom of start of 
    uint16_t pcstart;
    uint32_t cycles;

    unsigned done : 1;
    unsigned error : 1;
    unsigned use_lr : 1; // (unused, actually)
    
    // 0: leave as-is
    // 1: clear
    // 2: set
    unsigned edi : 2;
    
    // 1 if (main branch) epilogue inserted already.
    unsigned epilogue : 1;
    
    size_t next_label;
    
    // points to cbz/cbnz instructions, to backpatch labels into afterward.
    uint16_t* labelmap[JIT_MAX_LABELS];
} dis;

static uint8_t sm83_next_byte(void)
{
    dis.cycles += 1;
    return *(dis.rom++);
}

static uint16_t sm83_next_word(void)
{
    uint16_t x = sm83_next_byte();
    x |= sm83_next_byte() << 8;
    return x;
}

static uint8_t reg_cycles(sm83_oparg_t reg)
{
    // note that reading immediate values costs 1 cycle per byte,
    // and that is done separately
    
    switch (reg)
    {
    case OPARG_BCm:
    case OPARG_DEm:
    case OPARG_HLm:
    case OPARG_HLmd:
    case OPARG_HLmi:
    case OPARG_Cm:
    case OPARG_i8m:
    case OPARG_i16m:
        return 1;
    default:
        return 0;
    }
}

// is high 8 bits of 16-bit register (B, H, D)
static int is_reghi(sm83_oparg_t reg)
{
    switch(reg)
    {
    case OPARG_B:
    case OPARG_H:
    case OPARG_D:
        return 1;
    default:
        return 0;
    }
}

// is low 8 bits of 16-bit register (C, E, L)
static int is_reglo(sm83_oparg_t reg)
{
    switch(reg)
    {
    case OPARG_C:
    case OPARG_E:
    case OPARG_L:
        return 1;
    default:
        return 0;
    }
}

// is 8-bit register (A, B, C, D, E, H, L)
static int is_reg8(sm83_oparg_t reg)
{
    switch(reg)
    {
    case OPARG_A:
    case OPARG_B:
    case OPARG_C:
    case OPARG_D:
    case OPARG_E:
    case OPARG_H:
    case OPARG_L:
        return 1;
    default:
        return 0;
    }
}

static int is_reg16(sm83_oparg_t reg)
{
    switch(reg)
    {
    case OPARG_BC:
    case OPARG_DE:
    case OPARG_HL:
    case OPARG_SP:
        return 1;
    default:
        return 0;
    }
}

static int reg_armidx(sm83_oparg_t reg)
{
    switch(reg)
    {
    case OPARG_A:
        return REG_A;
    case OPARG_B:
    case OPARG_C:
    case OPARG_BC:
    case OPARG_BCm:
        return REG_BC;
    case OPARG_D:
    case OPARG_E:
    case OPARG_DE:
    case OPARG_DEm:
        return REG_DE;
    case OPARG_H:
    case OPARG_L:
    case OPARG_HL:
    case OPARG_HLm:
        return REG_HL;
    case OPARG_SP:
        return REG_SP;
    default:
        assert(false);
        return 0;
    }
}

static uint32_t swap16_32(uint32_t a)
{
    return ((a >> 16) & 0x0000ffff) | ((a << 16) & 0xffff0000);
}

#define WRITE_BUFF_INIT() uint16_t* const outbuff_base = outbuff;
#define WRITE_BUFF_16(arg) do { if (outbuff_base) *(uint16_t*)(void*)outbuff=(arg); outbuff += 1; } while(0)
#define WRITE_BUFF_32(arg) do { if (outbuff_base) *(uint32_t*)(void*)outbuff=swap16_32(arg); outbuff += 2; } while(0)
#define FWD_BUFF() (outbuff_base ? outbuff : outbuff_base)
#define WRITE_BUFF_END() ((uintptr_t)outbuff - (uintptr_t)outbuff_base) / sizeof(*outbuff)

static uint8_t fasm_skip(void* args, uint16_t* outbuff)
{
    return 0;
}

static uint8_t fasm_16(void* args, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    WRITE_BUFF_16((uintptr_t)args);
    return WRITE_BUFF_END();
}

static uint8_t fasm_32(void* args, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    WRITE_BUFF_32((uintptr_t)args);
    return WRITE_BUFF_END();
}

static uint8_t fasm_cmpbranch(void* argsv, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    uintptr_t args = (uintptr_t)argsv;
    uint8_t label = bits(args, 0, 8);
    bool n = bit(args, 8);
    uint8_t reg = bits(args, 16, 8);
    
    assert(reg == (reg & 0b111));
    assert(label < JIT_MAX_LABELS);
    dis.labelmap[label] = outbuff;
    
    // we'll come back and patch in the difference later.
    const uint16_t op =
        (n ? THUMB16_CBNZ : THUMB16_CBZ)
        | reg;
    WRITE_BUFF_16(op);
    return WRITE_BUFF_END();
}

static uint8_t fasm_backpatch(void* labelv, uint16_t* outbuff)
{
    if (outbuff)
    {
        uint8_t label = (uintptr_t)labelv;
        assert(label < JIT_MAX_LABELS);
        uint16_t* base = dis.labelmap[label];
        assert(base != NULL);
        size_t offset = (char*)outbuff - (char*)base;
        
        /*
            FIXME: why must we subtract 4? Instruction size is 2..?
        */
        assert (offset >= 4);
        offset -= 4;
        
        assert((offset & 1) == 0);
        offset >>= 1;
        assert(offset < 0x20);
        *base |= bits(offset, 0, 5) << 3;
        *base |= bit(offset, 5) << 9;
    }
    
    return 0;
}

#ifdef JIT_DEBUG
    #define FRAG(...) ({frag_t op = {__VA_ARGS__}; op.romsrc = NULL; op.length = 0; op;})
#else
    #define FRAG(...) ({frag_t op = {__VA_ARGS__}; op;})
#endif

static void frag_instr16(uint16_t instr)
{
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)(uintptr_t)instr,
        .produce = fasm_16
    );
}

static void frag_cmpbranch(bool cbnz, uint8_t reg, uint8_t label)
{
    assert(label < JIT_MAX_LABELS);
    const uintptr_t arg = (label) | (((uintptr_t)reg) << 16) | ((uintptr_t)cbnz << 8);
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)(uintptr_t)arg,
        .produce = fasm_cmpbranch
    );
}

static void frag_label(uint8_t label)
{
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)(uintptr_t)label,
        .produce = fasm_backpatch
    );
}

static void frag_instr32(uint32_t instr)
{
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)(uintptr_t)instr,
        .produce = fasm_32
    );
}

static uint32_t bitsmear_right(uint32_t b)
{
    b = b | (b >> 1);
    b = b | (b >> 2);
    b = b | (b >> 4);
    b = b | (b >> 8);
    b = b | (b >> 16);
    return b;
}

static uint32_t bitsmear_left(uint32_t b)
{
    b = b | (b << 1);
    b = b | (b << 2);
    b = b | (b << 4);
    b = b | (b << 8);
    b = b | (b << 16);
    return b;
}

// turns all bits to 1 except for leading and trailing 0s.
// e.g. converts 0b0010100100 to 0b0011111100.
static uint32_t contiguous_bits(uint32_t b)
{
    return bitsmear_left(b) & bitsmear_right(b);
}

// address is "safe" to write into if it won't cause a bankswap.
// if a bankswap could occur when writing to this address, then we have to
// stop dynarec after this instruction because we could suddenly be in a different bank.
// (admittedly, it's extremely unlikely to switch the active bank, but it could happen.)
static int sm83_addr_safe_write(uint16_t addr)
{
    if (opts.fixed_bank) return true;
    if (addr >= 0x200 && addr < 0x6000) return false;
    return true;
}

static uint8_t fasm_bl(void* fn, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    
    const uint32_t base_addr_none = (uintptr_t)arm_interworking_none(fn);
    const uint32_t base_addr_thumb = (uintptr_t)arm_interworking_thumb(fn);
    
    #ifdef TARGET_QEMU
    
    assert(sizeof(void*) == sizeof(uint32_t));
    const uint32_t imm32 = base_addr_thumb;
    
    // movw
    const unsigned dstidx = 14;
    const uint16_t immlo = imm32 & 0xFFFFFFFF;
    WRITE_BUFF_32(
        0xf2400000
        | (dstidx << 8)
        | (bits(immlo, 0, 8) << 0)
        | (bits(immlo, 8, 3) << 12)
        | (bits(immlo, 11, 1) << 26)
        | (bits(immlo, 12, 4) << 16)
    );
    
    // movt
    const uint16_t immhi = imm32 >> 16;
    WRITE_BUFF_32(
        0xf2c00000
        | (dstidx << 8)
        | (bits(immhi, 0, 8) << 0)
        | (bits(immhi, 8, 3) << 12)
        | (bits(immhi, 11, 1) << 26)
        | (bits(immhi, 12, 4) << 16)
    );
    
    // blx lr
    WRITE_BUFF_16(
        0x4780 | (14 << 3)
    );
    
    #else
    
    // bl fn
    int32_t rel_addr = (base_addr_none - (intptr_t)outbuff) / 2 - 2;
    uint32_t urel_addr = rel_addr;
    uint32_t sign = rel_addr < 0
        ? (1 << 26)
        : 0;
    uint32_t j2 = (!(urel_addr & (1 << 20)) == !sign)
        ? 0x800
        : 0x0000;
    uint32_t j1 = (!(urel_addr & (1 << 21)) == !sign)
        ? 0x2000
        : 0x0000;
    
    #ifdef TARGET_PLAYDATE
    if (outbuff)
    {
        if ((int)urel_addr < -16777216 || (int)urel_addr > 16777214)
        {
            // shouldn't be possible -- playdate's memory is only 16 mb.
            JIT_DEBUG_MESSAGE("bl out of range... %lx from %lx to %lx", abs(urel_addr), outbuff, fn);
            assert(false);
        }
    }
    #endif
    
    //JIT_DEBUG_MESSAGE("callsite address: %8x", outbuff);
    //JIT_DEBUG_MESSAGE("function address: %8x", fn);
    //JIT_DEBUG_MESSAGE("relative addr: %8x; sign %x, j2 %x, j1 %x\n", urel_addr, sign, j2, j1);
    
    WRITE_BUFF_32(
        0xF000D000 | (urel_addr & 0x7ff) | ((urel_addr << 5) & 0x03ff0000) | j2 | j1 | sign
    );
    #endif
    
    return WRITE_BUFF_END();
}

static void frag_bl(fn_type fn)
{
    dis.use_lr = 1;
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)fn,
        .produce = fasm_bl
    );
}

static void frag_delegate(fn_type fn)
{
    uint32_t reg_push = REGS_NONFLEX & 0xF;
    
    if (reg_push)
    {
        frag_instr16(reg_push | 0xb400);
    }
    
    frag_bl(fn);
    
    if (reg_push)
    {
        frag_instr16(reg_push | 0xbc00);
    }
}

// see ThumbExpand_C
static unsigned thumbexpand_encoding(uint8_t value, unsigned ror)
{
    if (ror == 0 || value == 0)
    {
        return value;
    }
    else
    {
        int hb = __builtin_clz(value) - __builtin_clz(0x100);
        value <<= hb;
        return ((hb + ror-1) << 7) | (value >> 1);
    }
}

// insert bits
static void frag_bfi(
    uint8_t rd,
    uint8_t rn,
    uint8_t lsb,
    uint8_t width
)
{
    uint8_t msb = (width+lsb-1);
    frag_instr32(
        THUMB32_BFI
        | (rd << 8)
        | (rn << 16)
        | (msb << 0)
        | (bits(lsb, 0, 2) << 6)
        | (bits(lsb, 2, 3) << 12)
    );
}

static void frag_instr16_rd0_rm3(
    uint16_t instruction,
    uint8_t rd,
    uint8_t rm
)
{
    assert( rd == (rd & 0x7) );
    assert( rm == (rm & 0x7) );
    frag_instr16(
        instruction | rd | (rm << 3)
    );
}

static void frag_instr16_rd0_rn3_rm6(
    uint16_t instruction,
    uint8_t rd,
    uint8_t rn,
    uint8_t rm
)
{
    assert( rd == (rd & 0x7) );
    assert( rn == (rn & 0x7) );
    assert( rm == (rm & 0x7) );
    frag_instr16(
        instruction | rd | (rn << 3) | (rm << 6)
    );
}

static void frag_instr16_rd0_rm3_imm5(
    uint16_t instruction,
    uint8_t rd,
    uint8_t rm,
    uint8_t imm5
)
{
    assert( rd == (rd & 0x7) );
    assert( rm == (rm & 0x7) );
    frag_instr16_rd0_rm3(
        instruction | (imm5 << 6), rd, rm
    );
}

static void frag_instr16_rd0_imm8(
    uint16_t instruction,
    uint8_t rd,
    uint8_t imm8
)
{
    assert( rd == (rd & 0x7) );
    frag_instr16(
        instruction | imm8 | (rd << 8)
    );
}

static void frag_uxtb(
    uint8_t dst, uint8_t src
)
{
    if (dst <= 7 && src <= 7)
    {
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, dst, src
        );
    }
    else
    {
        frag_instr32(
            THUMB32_UXTB
            | src
            | (dst << 8)
        );
    }
}

static void frag_uxth(
    uint8_t dst, uint8_t src
)
{
    if (dst <= 7 && src <= 7)
    {
        frag_instr16_rd0_rm3(
            THUMB16_UXTH, dst, src
        );
    }
    else
    {
        frag_instr32(
            THUMB32_UXTH
            | src
            | (dst << 8)
        );
    }
}

static void frag_armpush(uint16_t regs)
{
    if (bit(regs, 15) || bit(regs, 13))
    {
        assert(false);
    }
    
    if (regs == 0) return;
    
    if (!bits(regs, 8, 4))
    {
        if (regs & (1 << 14))
        {
            frag_instr16((regs & 0xff) | 0xb500);
        }
        else
        {
            frag_instr16(regs | 0xb400);
        }
    }
    else
    {
        if (__builtin_popcount(regs) >= 2)
        {
            frag_instr32(regs | 0xe92d0000);
        }
        else
        {
            assert(__builtin_popcount(regs) == 1);
            uint8_t reg = __builtin_ctz(regs);
            // T3
            frag_instr32(
                0xf84d0d04
                | (reg << 12)
            );
        }
    }
}

static void frag_armpop(uint16_t regs)
{
    if (bit(regs, 15) || bit(regs, 13))
    {
        assert(false);
    }
    
    if (regs == 0) return;
    
    if (!bits(regs, 8, 4) && !bit(regs, 14))
    {
        // pop { regs }
        frag_instr16(regs | 0xbc00);
    }
    else
    {
        if (__builtin_popcount(regs) == 1)
        {
            if (bit(regs, 14))
            {
                // pop { lr }
                frag_instr32(0xF85DEB04);
            }
            else
            {
                // TODO
                assert(false);
            }
        }
        else
        {
            // pop { regs }
            frag_instr32(regs | 0xE8BD4000);
        }
    }
}

// moves 16-bit immediate into the given register
// no assumptions are made about the contents of dst
static void frag_ld_imm16(int dstidx, uint16_t imm)
{
    if (imm <= 0xff && (dstidx == (dstidx & 0x07)))
    {
        // MOVS r%dst, imm [T1]
        frag_instr16(
            0x2000
            | (imm)
            | (dstidx << 8)
        );
    }
    else
    {
        // MOVW r%dst, imm16 [T3]
        frag_instr32(
            0xf2400000
            | (dstidx << 8)
            | (bits(imm, 0, 8) << 0)
            | (bits(imm, 8, 3) << 12)
            | (bits(imm, 11, 1) << 26)
            | (bits(imm, 12, 4) << 16)
        );
    }
}

static void frag_ld_imm32(int dstidx, uint32_t imm)
{
    frag_ld_imm16(dstidx, imm & 0xFFFF);
    uint16_t immt = imm >> 16;
    
    // MOVT r%dst, imm16
    frag_instr32(
        0xf2c00000
        | (dstidx << 8)
        | (bits(imm, 0, 8) << 0)
        | (bits(imm, 8, 3) << 12)
        | (bits(imm, 11, 1) << 26)
        | (bits(imm, 12, 4) << 16)
    );
}

// writeback: ptrreg is modified (equivalent to '!' in assembly)
static void frag_access_multiple(bool store, uint16_t regs, uint8_t ptrreg, bool writeback)
{
    const uint16_t op16 = (store)
        ? THUMB16_STM
        : THUMB16_LDM;
    
    const uint32_t op32 = (store)
        ? THUMB32_STM
        : THUMB32_LDM;
    
    if (regs == (regs & 0xff) && writeback)
    {
        frag_instr16(
            op16 | regs | (ptrreg << 8)
        );
    }
    else
    {
        frag_instr32(
            op32 | regs | (ptrreg << 16) | (writeback << 14)
        );
    }
}

static void frag_mov_rd_rm(
    uint8_t rd, uint8_t rm
)
{
    if (rd >= 8 || rm >= 8)
    {
        frag_instr16(
            THUMB16_MOV_REG_T2
            | (rm << 3)
            | (bits(rd, 0, 3))
            | (bit(rd, 3) << 7)
        );
    }
    else
    {
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, rd, rm
        );
    }
}

static void frag_ubfx(
    uint8_t rd,
    uint8_t rn,
    uint8_t pos,
    uint8_t width
)
{
    assert(width > 0);
    frag_instr32(
        0xf3c00000
        | (rd << 8)
        | (rn << 16)
        | ((width-1) & 0b11111)
        | (bits(pos, 0, 2) << 6)
        | (bits(pos, 2, 3) << 12)
    );
}

static void frag_sbfx(
    uint8_t rd,
    uint8_t rn,
    uint8_t pos,
    uint8_t width
)
{
    assert(width > 0);
    frag_instr32(
        0xf3400000
        | (rd << 8)
        | (rn << 16)
        | ((width-1) & 0b11111)
        | (bits(pos, 0, 2) << 6)
        | (bits(pos, 2, 3) << 12)
    );
}

#define IMM_SHIFT_LSL 0
#define IMM_SHIFT_LSR 1
#define IMM_SHIFT_ASR 2
#define IMM_SHIFT_ROR 3
#define IMM_SHIFT_RRX 4

static void frag_rd8_rn16_rm0_shift(
    uint32_t instruction,
    uint8_t rd,
    uint8_t rn,
    uint8_t rm,
    uint8_t shift_type,
    uint8_t shift_x
)
{
    if (shift_type == IMM_SHIFT_RRX)
    {
        assert(shift_x == 0);
        shift_type = IMM_SHIFT_ROR;
    }
    if (shift_type != IMM_SHIFT_LSL)
    {
        assert(shift_x != 0);
    }
    
    frag_instr32(instruction
        | (rm)
        | (rd << 8)
        | (rn << 16)
        | (shift_type << 4)
        | (bits(shift_x, 0, 2) << 6)
        | (bits(shift_x, 2, 3) << 12)
    );
}

static void frag_imm12c_rd8_rn16(
    uint32_t instruction,
    uint8_t rd,
    uint8_t rn,
    bool setflags,
    uint8_t value,
    uint8_t ror
)
{
    const uint32_t encoding = thumbexpand_encoding(value, ror);
    frag_instr32(
        instruction
        | (rd << 8)
        | (rn << 16)
        | (setflags << 20)
        | bits(encoding, 0, 8)
        | (bits(encoding, 8, 3) << 12)
        | (bit(encoding, 11) << 26)
    );
}

static void frag_add_rd_rn(
    uint8_t rdn, uint8_t rm
)
{
    frag_instr16(
        THUMB16_ADD_REG_T2
        | (rm << 3)
        | (bits(rdn, 0, 3))
        | (bit(rdn, 3) << 7)
    );
}

static void frag_add_imm16(
    uint8_t rd, uint8_t rn,
    int32_t imms
)
{
    if (imms == 0) return;
    
    uint8_t sub = false;
    uint16_t imm = imms;
    if (imms < 0)
    {
        sub = true;
        imm = -imms;
    }
    
    assert(imm == (imm & 0xffff));
    
    if (imm == (imm & 0b111) && rn == (rn & 0b111) && rd == (rd & 0b111))
    {
        frag_instr16(
            (sub ? THUMB16_SUB_3 : THUMB16_ADD_3)
            | rd | (rn << 3) | (imm << 6)
        );
    }
    else if (imm == (imm & 0xff) && rd == rn && rd == (rd & 0b111))
    {
        frag_instr16(
            (sub ? THUMB16_SUB_8 : THUMB16_ADD_8)
            | (rd << 8) | imm
        );
    }
    else
    {
        frag_instr32(
            (sub ? THUMB32_SUB_IMM_T4 : THUMB32_ADD_IMM_T4)
            | (rd << 8)
            | (rn << 16)
            | bits(imm, 0, 8)
            | (bits(imm, 8, 3) << 12)
            | (bits(imm, 11, 1) << 26)
        );
    }
}

// epilogue: cleans up and (arm) returns.
// pc: set sm83 program counter to this value
//   - if pc is -1, pc <- hl
//   - if pc is -2, pop pc from sm83 stack
// cycles: return this value in r0
static void frag_epilogue(int32_t pc, uint32_t cycles)
{    
    assert(cycles < 0x10000);
    
    // pop {r0}
    frag_armpop(1 << 0);
    
    uint16_t stm_regs = REGS_SM83;
    if (pc == -2) stm_regs &= ~(1 << REG_SP);
    
    // STM r%regf, { ... }
    frag_access_multiple(
        true, stm_regs, 0, false
    );
    
    if (dis.edi)
    {
        frag_ld_imm16(1, dis.edi - 1);
        
        // str r0, [r%regf, #jit_regfile_t.ime]
        frag_instr16_rd0_rm3_imm5(
            THUMB16_STR,
            0,
            1,
            offsetof(jit_regfile_t, ime) >> 2
        );
    }
    
    // store pc
    uint8_t reg_regfile = 0;
    uint8_t reg_pc = 1;
    if (pc >= 0)
    {
        assert(pc < 0x10000);
        frag_ld_imm16(reg_pc, pc);
    }
    else if (pc == -1)
    {
        frag_mov_rd_rm(reg_pc, REG_HL);
    }
    else if (pc == -2)
    {
        const uint8_t reg_sp_tmp = 5;
        reg_pc = 0;
        reg_regfile = 4;
        
        // assert these are callee-pushed by the arm chunk.
        assert(REGS_NONFLEX & (1 << reg_regfile));
        assert(REGS_NONFLEX & (1 << reg_sp_tmp));
        
        frag_mov_rd_rm(reg_regfile, 0);
        if (true) //(dis.use_sp)
        {
            frag_mov_rd_rm(0, REG_SP);
        }
        else
        {
            // ldr r0, [r0, #jit_regfile_t.sp]
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LDR,
                0,
                0,
                offsetof(jit_regfile_t, sp) >> 2
            );
        }
        frag_mov_rd_rm(reg_sp_tmp, 0);
        
        frag_bl((fn_type)opts.readword);
        
        // sp += 2
        frag_add_imm16(reg_sp_tmp, reg_sp_tmp, 2);
        
        // (no need to uxth because we strh.)
        frag_instr16_rd0_rm3_imm5(
            THUMB16_STRH,
            reg_sp_tmp,
            reg_regfile,
            offsetof(jit_regfile_t, sp) >> 1
        );
    }
    else
    {
        assert(false);
    }
    
    frag_instr16_rd0_rm3_imm5(
        THUMB16_STR,
        reg_pc,
        reg_regfile,
        offsetof(jit_regfile_t, pc) >> 2
    );
    
    // pop {...}
    frag_armpop((REGS_ALL & ~0xF) | (/*dis.use_lr*/1 << 14));
    
    frag_ld_imm16(0, cycles);
    
    // bx lr
    frag_instr16(0x4770);
    
    // TODO: figure out when this is needed exactly.
    // nop
    frag_instr16(0xbf00);
}

// results in r1-r3 pushed, and r0 containing the read of (hl)
static void frag_pre_readwrite_hlm(void)
{
    assert(REG_HL >= 4); // need this to be able to copy hl to r0 for argument.
    frag_armpush(REGS_NONFLEX & 0xf);
    
    // movs r0, r%hl
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_HL);
    
    frag_bl((fn_type)opts.read);
}

static void frag_read_hlm(void)
{
    assert(REG_HL >= 4); // need this to be able to copy hl to r0 for argument.
    assert (! (REGS_NONFLEX & 0x1));
    frag_armpush(REGS_NONFLEX & 0xf);
    
    // movs r0, r%hl
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_HL);
    
    frag_bl((fn_type)opts.read);
    
    frag_armpop(REGS_NONFLEX & 0xf);
}

// writes r1 to (hl), then pops r1-r3.
static void frag_post_readwrite_hlm(void)
{
    // movs r0, r%hl
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_HL);
    
    frag_bl((fn_type)opts.write);
    
    frag_armpop(REGS_NONFLEX & 0xF);
    
    // (too improbable -- discounting this possibility.)
    #if 0
        // we have to stop if we write to rom, for fear of bank swap
        if (!opts.fixed_bank) dis.done = 1;
    #endif
}

static void frag_set_n(bool n)
{
    frag_imm12c_rd8_rn16(
        n ? THUMB32_ORR_IMM : THUMB32_BIC_IMM,
        REG_NH, REG_NH, false, 0x20, 0
    );
}

static void frag_set_nh(bool n, bool h)
{
    const uint16_t value = (n << 5) | (h << 12);
    frag_ld_imm16(REG_NH, value);
}

// v: 0 if clear, 1 if set.
// b: the bit to set (0-7)
static void frag_set(unsigned b, sm83_oparg_t dst, int v)
{
    dis.cycles += reg_cycles(dst)*2;
    
    uint32_t instruction = (v)
        // orr.w
        ? THUMB32_ORR_IMM
        // bic
        : THUMB32_BIC_IMM;
    
    if (dst == OPARG_HLm)
    {
        frag_pre_readwrite_hlm();
        
        // bic/orr r1,r0,#(1<<b)
        frag_imm12c_rd8_rn16(instruction, 0, 1, false, 1, 32-b);
        
        frag_post_readwrite_hlm();
    }
    else if (is_reg8(dst))
    {
        if (is_reghi(dst))
        {
            frag_imm12c_rd8_rn16(instruction, reg_armidx(dst), reg_armidx(dst), false, 1, 24-b);
        }
        else
        {
            frag_imm12c_rd8_rn16(instruction, reg_armidx(dst), reg_armidx(dst), false, 1, 32-b);
        }
    }
    else
    {
        assert(false);
    }
}

static void frag_cpl(void)
{
    // eor r%a, #$ff
    frag_imm12c_rd8_rn16(
        THUMB32_EOR_IMM,
        REG_A, REG_A, 0, 0xff, 0
    );
    
    frag_set_nh(true, true);
}

static void frag_incdec(sm83_oparg_t arg, bool dec)
{
    int offset = (dec) ? -1 : 1;
    const uint16_t op = (dec)
        ? 0x3801
        : 0x3001;
        
    const uint32_t op32 = (dec)
        ? 0xf1a00000
        : 0xf5000000;
    
    if (is_reg16(arg))
    {
        // adds r%arg, 1 [T2]
        frag_add_imm16(reg_armidx(arg), reg_armidx(arg), offset);
        
        // uxth r%arg, r%arg
        frag_uxth(reg_armidx(arg), reg_armidx(arg));
        
        // [sic] flags are unaffected
    }
    else if (is_reg8(arg))
    {
        if (arg == OPARG_A)
        {
            #if SETFLAGS
            if (!dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, REG_A, 16
                );
            }
            #endif
            
            // adds/subs r%arg, 1 [T2]
            frag_add_imm16(reg_armidx(arg), reg_armidx(arg), offset);
            
            #if SETFLAGS
            if (dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, REG_A, 24
                );
            }
            #endif
            
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, REG_A, REG_A
            );
            
            #if SETFLAGS
            frag_imm12c_rd8_rn16(
                THUMB32_ORR_IMM, REG_NH, REG_NH, false, 0x10 + dec, 28
            );
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_A);
            #endif
        }
        else if (is_reglo(arg))
        {
            // lsrs r0, r%arg, #8
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, 0, reg_armidx(arg), 8);
            
            #if SETFLAGS
            if (!dec)
            {
                frag_instr16_rd0_rm3(
                    THUMB16_UXTB, REG_NH, reg_armidx(arg)
                );
            }
            #endif
            
            // adds/subs r%arg, 1 [T2]
            frag_add_imm16(reg_armidx(arg), reg_armidx(arg), offset);
            
            #if SETFLAGS
            if (!dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, REG_NH, 16
                );
            }
            else
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, reg_armidx(arg), 24
                );
            }
            #endif
            
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, reg_armidx(arg), reg_armidx(arg)
            );
            
            #if SETFLAGS
            frag_imm12c_rd8_rn16(
                THUMB32_ORR_IMM, REG_NH, REG_NH, false, 0x10 + dec, 28
            );
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_A);
            #endif
            
            // orr.w r%arg, r%arg, r0<<8
            frag_instr32(
                0xea402000
                | (reg_armidx(arg) << 16)
                | (reg_armidx(arg) << 8)
                | (0 << 0)
            );
        }
        else if (is_reghi(arg))
        {
            #if SETFLAGS
            if (!dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSR_IMM, REG_NH, reg_armidx(arg), 8
                );
            }
            #endif
            
            // adds/subs r%arg, r%arg, #$100
            frag_add_imm16(reg_armidx(arg), reg_armidx(arg), offset << 8);
            
            #if SETFLAGS
            if (dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSR_IMM, REG_NH, reg_armidx(arg), 8
                );
            }
            else
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, REG_NH, 16
                );
            }
            #endif
            
            // uxth r%arg, r%arg
            frag_instr16(0xb280 | (reg_armidx(arg) << 3) | reg_armidx(arg));
            
            #if SETFLAGS
            if (dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, REG_NH, 24
                );
            }
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSR_IMM, REG_nZ, reg_armidx(arg), 8
            );
            frag_imm12c_rd8_rn16(
                THUMB32_ORR_IMM, REG_NH, REG_NH, false, 0x10 + dec, 28
            );
            #endif
        }
    }
    else if (arg == OPARG_HLm)
    {
        frag_pre_readwrite_hlm();
        
        #if SETFLAGS
        if (!dec)
        {
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSL_IMM, REG_NH, 0, 24
            );
        }
        #endif
        
        // adds/subs r0,r0,#1 [T2]
        frag_add_imm16(0, 0, offset);
        
        #if SETFLAGS
        if (dec)
        {
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSL_IMM, REG_NH, 0, 24
            );
        }
        #endif
        
        // uxtb r1, r0
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, 1, 0
        );
        
        #if SETFLAGS
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, REG_nZ, 0
        );
        frag_imm12c_rd8_rn16(
            THUMB32_ORR_IMM, REG_NH, REG_NH, false, 0x10 + dec, 28
        );
        #endif
        
        frag_post_readwrite_hlm();
    }
    else
    {
        assert(false);
    }
}

static void frag_inc(sm83_oparg_t arg)
{
    frag_incdec(arg, false);
}

static void frag_dec(sm83_oparg_t arg)
{
    frag_incdec(arg, true);
}

// moves 8-bit sm83 register into r0 or r%a
// dstidx must be the index of an arm register whose upper 24 bits can be assumed to be 0.
static void frag_store_rz_reg8(int dstidx, sm83_oparg_t src)
{
    assert(dstidx == REG_A || dstidx == REG_FLEX);
    assert(is_reg8(src));
    
    const uint8_t rn = reg_armidx(src);
    const uint8_t rd = dstidx;
    
    if (src == OPARG_A)
    {
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, dstidx, REG_A
        );
    }
    else if (is_reghi(src))
    {
        // lsrs r%dst, r%src, 8
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, rd, rn, 8
        );
    }
    else
    {
        frag_bfi(rd, rn, 0, 8);
    }
}

// moves r0 or r%a into 8-bit sm83 register
// srcidx must be the index of an arm register whose upper 24 bits can be assumed to be 0.
static void frag_store_reg8_rz(int srcidx, sm83_oparg_t dst)
{
    assert(srcidx == REG_A || srcidx == REG_FLEX);
    assert(is_reg8(dst));
    
    const uint8_t rd = reg_armidx(dst);
    const uint8_t rn = srcidx;
    
    if (dst == OPARG_A)
    {
        if (srcidx != REG_A)
        {
            // movs r%a, r0 [T2]
            frag_instr16(
                0x0 | rd | (rn << 3)
            );
        }
    }
    else if (is_reglo(dst))
    {
        frag_bfi(rd, rn, 0, 8);
    }
    else if (is_reghi(dst))
    {
        frag_bfi(rd, rn, 8, 8);
    }
    else
    {
        assert(false);
    }
}

// dst <- r%sp + <signed 8-byte immediate>
// used in:
//   ld hl, sp+<signed i8>
//   add sp, <signed i8>
static void frag_spi8(sm83_oparg_t dst)
{
    // FIXME: double check flags here.
    dis.cycles += 1;
    if (dst == OPARG_SP) dis.cycles += 1;
    
    assert(is_reg16(dst));
    
    int8_t immediate = sm83_next_byte();
    
    #if SETFLAGS
    // r%c <- r%sp & 0xff
    frag_uxtb(REG_Carry, REG_SP);
    
    // nh[3] <- r%sp << 24
    frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_NH, REG_Carry, 24);
    
    // r%c += immediate
    frag_add_imm16(REG_Carry, REG_Carry, immediate & 0xff);
    
    // nh[2] <- immediate
    frag_imm12c_rd8_rn16(
        THUMB32_ORR_IMM,
        REG_NH, REG_NH, false, (uint8_t)immediate, 16
    );
    
    // r%c >>= 8
    frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, REG_Carry, 8);
    #endif
    
    frag_add_imm16(reg_armidx(dst), REG_SP, immediate);
    
    #if SETFLAGS
    // z <- 0
    frag_ld_imm16(REG_nZ, 1);
    #endif
    
    frag_uxth(reg_armidx(dst), reg_armidx(dst));
}

static void frag_ld(sm83_oparg_t dst, sm83_oparg_t src)
{
    if (src == OPARG_B && dst == OPARG_B && opts.ld_b_b)
    {
        // ld b,b is a breakpoint.
        frag_delegate(opts.ld_b_b);
        dis.done = 1;
        return;
    }
    else if (dst == OPARG_HLmi)
    {
        frag_ld(OPARG_HLm, src);
        frag_inc(OPARG_HL);
        return;
    }
    else if (dst == OPARG_HLmd)
    {
        frag_ld(OPARG_HLm, src);
        frag_dec(OPARG_HL);
        return;
    }
    else if (src == OPARG_HLmi)
    {
        frag_ld(dst, OPARG_HLm);
        frag_inc(OPARG_HL);
        return;
    }
    else if (src == OPARG_HLmd)
    {
        frag_ld(dst, OPARG_HLm);
        frag_dec(OPARG_HL);
        return;
    }
    else if (src == OPARG_i8sp)
    {
        frag_spi8(dst);
        return;
    }
    
    // cycles
    dis.cycles += reg_cycles(src);
    dis.cycles += reg_cycles(dst);
    
    unsigned immediate = 0;
    if (src == OPARG_i8 || src == OPARG_i8m || src == OPARG_i8sp
        || dst == OPARG_i8 || dst == OPARG_i8m || dst == OPARG_i8sp)
    {
        immediate = sm83_next_byte();
    }
    else if (src == OPARG_i16 || src == OPARG_i16m || dst == OPARG_i16 || dst == OPARG_i16m)
    {
        immediate = sm83_next_word();
    }

    if (src == OPARG_BCm || src == OPARG_DEm || src == OPARG_HLm
        || src == OPARG_i16m || src == OPARG_i8m || src == OPARG_Cm)
    {
        assert(is_reg8(dst));
        unsigned regs = REGS_NONFLEX & 0xF;
        
        // we can skip preserving A if we're going to write to it.
        if (dst == OPARG_A)
        {
            regs &= ~(1<<REG_A);
        }
        
        frag_armpush(regs);
        
        switch (src)
        {
        case OPARG_BCm:
        case OPARG_DEm:
        case OPARG_HLm:
            // movs r0, r%src
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, reg_armidx(src));
            break;
        case OPARG_i8m:
            immediate |= 0xff00;
            // fallthrough
        case OPARG_i16m:
            frag_ld_imm16(0, immediate);
            break;
        case OPARG_Cm:
            // orr.w r0, r%bc, #$ff00
            frag_instr32(
                0xf440407f
                | (REG_BC << 16)
            );
            break;
        default:
            assert(false);
        }
        
        // call read
        frag_bl((fn_type)opts.read);
        
        // pop {...}
        frag_armpop(regs);
        
        // transfer r0 to dst
        frag_store_reg8_rz(0, dst);
    }
    else if (src == OPARG_SP && dst == OPARG_i16m)
    {
        dis.cycles += 2;
        frag_armpush(REGS_NONFLEX & 0xF);
        
        frag_mov_rd_rm(0, REG_HL);
        frag_mov_rd_rm(1, REG_SP);
        
        // call write
        frag_bl((fn_type)opts.writeword);
        
        frag_armpop(REGS_NONFLEX);
    }
    else if (dst == OPARG_SP && src == OPARG_HL)
    {
        dis.cycles += 1;
        frag_mov_rd_rm(OPARG_SP, OPARG_HL);
    }
    else if (dst == OPARG_HL && src == OPARG_i8sp)
    {
        // this is to be handled by the frag_spi8 instruction
        assert(false);
    }
    else if (
        dst == OPARG_i16m || dst == OPARG_i8m || dst == OPARG_Cm
        || dst == OPARG_BCm || dst == OPARG_DEm || dst == OPARG_HLm
    )
    {
        assert(is_reg8(src) || src == OPARG_i8);
        // FIXME: frag_push/frag_pop should be its own instruction, because
        // if we clobber r1 and it's smear-written later... aaahh...
        // well, we always push r1 as a result, but that isn't *always* needed!
        
        frag_armpush(REGS_NONFLEX & 0xF);
        
        switch (dst)
        {
        case OPARG_BCm:
        case OPARG_DEm:
        case OPARG_HLm:
            // movs r0, r%dst
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, reg_armidx(dst));
            break;
        case OPARG_i8m:
            immediate |= 0xff00;
            // fallthrough
        case OPARG_i16m:
            frag_ld_imm16(0, immediate);
            break;
        case OPARG_Cm:
            // orr.w r0, r%bc, #$ff00
            frag_instr32(
                0xf440407f
                | (REG_BC << 16)
            );
            break;
        default:
            assert(false);
        }
        
        // movs r1, value
        assert (REG_A == 1);
        switch (src)
        {
        case OPARG_A:
            break; // nothing to be done.
        case OPARG_i8:
            frag_ld_imm16(1, immediate);
            break;
        default:
            // it's safe to clobber r1 with the write value
            // because we'll pop r1 -> a later if it is needed.
            frag_store_rz_reg8(1, src);
        }
        
        // call write
        frag_bl((fn_type)opts.write);
        
        // pop {...}
        frag_armpop(REGS_NONFLEX & 0xF);
        
        // (too unlikely -- discounting this possibility)
        #if 0
        // we have to stop now because of the possibility of a bankswap under our feet.
        if (!(dst == OPARG_i16m && sm83_addr_safe_write(immediate))
        || dst == OPARG_i8m || dst == OPARG_Cm
        || (opts.fixed_bank && (dst == OPARG_BCm || dst == OPARG_DEm || dst == OPARG_HLm)))
        {
            dis.done = true;
        }
        #else
        // vaguely plausible this could bankswap under our feet.
        if ((dst == OPARG_i8m || dst == OPARG_i16m) && !sm83_addr_safe_write(immediate))
        {
            dis.done = 1;
        }
        #endif
    }
    else if (is_reg8(dst))
    {
        if (src == OPARG_i8)
        // e.g. ld c, $50
        {
            if (dst == OPARG_A)
            {
                // movs r%a, imm
                // cf. ARMv7-M Architecture Reference Manual A6-148, Encoding T1
                frag_ld_imm16(reg_armidx(dst), immediate);
            }
            else
            {
                if (is_reghi(dst))
                {
                    // [A6.7.149] uxtb r%dst, r%dst
                    frag_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // [A6.7.90] orr  r%dst, r%dst, imm<<8
                    if (immediate != 0)
                    {
                        frag_imm12c_rd8_rn16(
                            THUMB32_ORR_IMM, reg_armidx(dst), reg_armidx(dst), 0, immediate, 24
                        );
                    }
                }
                else
                {
                    // and r%dst, $ff00
                    frag_imm12c_rd8_rn16(
                        THUMB32_AND_IMM, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                    
                    // orr  r%dst, imm
                    if (immediate != 0)
                    {
                        frag_imm12c_rd8_rn16(
                            THUMB32_ORR_IMM, reg_armidx(dst), reg_armidx(dst), 0, immediate, 0
                        );
                    }
                }
            }
        }
        else if (is_reg8(src))
        {
            if (src == dst) return;
            else if (dst == OPARG_A)
            {
                frag_store_rz_reg8(REG_A, src);
            }
            else if (src == OPARG_A)
            {
                frag_store_reg8_rz(REG_A, dst);
            }
            else // neither operand is A
            {
                if (reg_armidx(dst) == reg_armidx(src))
                {
                    if (is_reghi(src))
                    {                  
                        assert(is_reglo(dst));
                        // and r0, r%dst, #$ff00
                        frag_instr32(
                            0xf400407f
                            | (reg_armidx(dst) << 16)
                        );
                        
                        // orr r%dst, r0, r%dst, lsr #8
                        frag_rd8_rn16_rm0_shift(
                            THUMB32_ORR_REG, reg_armidx(dst), 0, reg_armidx(src), IMM_SHIFT_LSR, 8
                        );
                    }
                    else
                    {
                        assert(is_reglo(src));
                        assert(is_reghi(dst));
                        frag_bfi(reg_armidx(dst), reg_armidx(src), 8, 8);
                    }
                }
                else if (is_reghi(dst) && is_reghi(src))
                {
                    frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, 0, reg_armidx(src), 8);
                    frag_bfi(reg_armidx(dst), 0, 8, 8);
                }
                else if (is_reglo(dst) && is_reglo(src))
                {
                    frag_bfi(reg_armidx(dst), reg_armidx(src), 0, 8);
                }
                else if (is_reghi(dst) && is_reglo(src))
                {
                    frag_bfi(reg_armidx(dst), reg_armidx(src), 8, 8);
                }
                else if (is_reglo(dst) && is_reghi(src))
                {
                    // and r%dst, $ff00
                    frag_imm12c_rd8_rn16(
                        THUMB32_AND_IMM, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                    
                    // orr r%dst, r%dst, r%src, lsr #8
                    frag_instr32(
                        0xEA502010
                        | (reg_armidx(dst) << 16)
                        | (reg_armidx(dst) << 8)
                        | (reg_armidx(src) << 0)
                    );
                }
            }
        }
        else
        {
            assert(false);
        }
    }
    else if (is_reg16(dst))
    {
        if (src == OPARG_i16)
        // e.g. ld bc, $1234
        {
            frag_ld_imm16(reg_armidx(dst), immediate);
        }
        else
        {
            assert(false);
        }
    }
    else
    {
        assert(false);
    }
}

typedef enum
{
    SHIFT_LEFT,
    ROT_LEFT,
    ROT_LEFT_CIRCULAR,
    SHIFT_RIGHT,
    SHIFT_RIGHT_L,
    ROT_RIGHT,
    ROT_RIGHT_CIRCULAR,
} rotate_type;

static void frag_rotate_helper(rotate_type rt, uint8_t regdst, uint8_t reg, uint8_t carry_in, uint8_t carry_out, bool setz)
{
    const uint8_t scratchreg = (reg == 0) ? 1 : 0;
    switch (rt)
    {
    case SHIFT_LEFT:
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, reg, reg, 1
        );
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, regdst, reg
        );
        break;
    case ROT_LEFT:
        frag_bfi(reg, regdst, 1, 7);
        
        // orr r%reg, r%carry_in
        if (regdst == 0 && reg != regdst)
        {
            // the other branch would work too
            // it's just a guess that this might be faster?
            frag_instr16_rd0_rm3(
                THUMB16_ORR_REG, reg, carry_in
            );
        }
        else
        {
            frag_bfi(regdst, carry_in, 0, 1);
        }

        break;
    case ROT_LEFT_CIRCULAR:
    
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, reg, reg, 1
        );
        
        // reg |= reg >> 8
        frag_rd8_rn16_rm0_shift(
            THUMB32_ORR_REG, reg, reg, reg, IMM_SHIFT_LSR, 8
        );
        break;
    
    case SHIFT_RIGHT:
    case ROT_RIGHT:
    case ROT_RIGHT_CIRCULAR:
        if (rt == ROT_RIGHT_CIRCULAR)
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, reg, reg, reg, IMM_SHIFT_LSL, 8
            );
        }
        else if (rt == ROT_RIGHT)
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, reg, reg, carry_in, IMM_SHIFT_LSL, 8
            );
        }
        
        frag_ubfx(regdst, reg, 1, 8);
        break;
        
    case SHIFT_RIGHT_L:
        frag_sbfx(reg, reg, 1, 7);
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, regdst, reg
        );
        break;
    }
    
    if (setz)
    {
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, REG_nZ, regdst
        );
    }
    else
    {
        // z <- 0
        frag_ld_imm16(
            REG_nZ, 1
        );
    }
}

static void frag_rotate(rotate_type rt, sm83_oparg_t dst,  bool setz)
{
    // TODO: rotate can be vastly improved with the bfi instruction
    assert(is_reg8(dst) || dst == OPARG_HLm);
    
    dis.cycles += reg_cycles(dst) * 2;
    
    if (dst == OPARG_HLm)
    {
        frag_pre_readwrite_hlm();
    }
    
    uint8_t carry_in = REG_Carry;
    const uint8_t carry_out = REG_Carry;
    #if SETFLAGS
    const uint8_t bit_offset = is_reghi(dst)
        ? 8
        : 0;
    const uint8_t reg = (dst == OPARG_HLm) ? 0 : reg_armidx(dst);
    
    // transfer previous carry flag to nh;
    // this will effectively clear nh, and will allow us to briefly
    // rely on REG_NH containing the carry (in) bit,
    // which some rotations need to read (RL, RR).
    // we can then freely set the carry (out) in advance of the operation.
    carry_in = REG_NH;
    
    frag_instr16_rd0_rm3(
        THUMB16_MOV_REG, REG_NH, REG_Carry
    );
    
    // set carry out
    switch (rt)
    {
    case SHIFT_LEFT:
    case ROT_LEFT:
    case ROT_LEFT_CIRCULAR:
        frag_ubfx(REG_Carry, reg, 7+bit_offset, 1);
        break;
        
    case SHIFT_RIGHT:
    case SHIFT_RIGHT_L:
    case ROT_RIGHT:
    case ROT_RIGHT_CIRCULAR:
        frag_ubfx(REG_Carry, reg, 0+bit_offset, 1);
        break;
    }
    #endif
    
    if (dst == OPARG_A)
    {
        frag_rotate_helper(rt, REG_A, REG_A, carry_in, carry_out, setz);
    }
    else if (is_reglo(dst))
    {
        assert(setz);
        uint8_t scratch = 0;
        #if SETFLAGS
        scratch = REG_nZ;
        #endif
        
        switch(rt)
        {
        case SHIFT_LEFT:
            frag_bfi(reg, reg, 1, 7);
            frag_imm12c_rd8_rn16(THUMB32_BIC_IMM, reg, reg, false, 1, 0);
            #if SETFLAGS
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, REG_nZ, reg
            );
            #endif
            break;
        
        case ROT_LEFT:
        case ROT_LEFT_CIRCULAR:
            #if SETFLAGS
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, REG_nZ, reg
            );
            #endif
            frag_bfi(reg, reg, 1, 7);
            frag_bfi(reg, (rt == ROT_LEFT_CIRCULAR) ? carry_out : carry_in, 0, 1);
            break;
            
        case SHIFT_RIGHT:
            frag_ubfx(scratch, reg, 1, 7);
            frag_bfi(reg, scratch, 0, 8);
            break;
        
        case SHIFT_RIGHT_L:
            frag_sbfx(scratch, reg, 1, 7);
            frag_bfi(reg, scratch, 0, 8);
            break;
            
        case ROT_RIGHT:
        case ROT_RIGHT_CIRCULAR:
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSR_IMM, scratch, reg, 1
            );
            
            frag_bfi(scratch, (rt == ROT_RIGHT_CIRCULAR) ? carry_out : carry_in, 7, 1);
            frag_bfi(reg, scratch, 0, 8);
            break;
        }
    }
    else if (is_reghi(dst))
    {
        uint8_t scratch = 0;
        #if SETFLAGS
        scratch = REG_nZ;
        #endif
        assert(setz);
        switch (rt)
        {
        case SHIFT_LEFT:
            frag_ubfx(scratch, reg_armidx(dst), 8, 7);
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, reg_armidx(dst), reg_armidx(dst)
            );
            frag_bfi(reg_armidx(dst), scratch, 9, 7);
            break;
        case ROT_LEFT:
        case ROT_LEFT_CIRCULAR:
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSR_IMM, scratch, reg_armidx(dst), 7
            );
            frag_instr16_rd0_rm3(
                THUMB16_ORR_REG, scratch, (rt == ROT_LEFT_CIRCULAR) ? carry_out : carry_in
            );
            
            #if SETFLAGS
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, scratch, scratch
            );
            #endif
            
            frag_bfi(reg_armidx(dst), scratch, 8, 8);
            break;
        case ROT_RIGHT_CIRCULAR:
        case ROT_RIGHT:
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSR_IMM, scratch, reg_armidx(dst), 9
            );
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, scratch, scratch, (rt == ROT_RIGHT_CIRCULAR) ? carry_out : carry_in, IMM_SHIFT_LSL, 7
            );
            frag_bfi(reg_armidx(dst), scratch, 8, 8);
            break;
        case SHIFT_RIGHT:
            frag_instr16_rd0_rm3_imm5(
                THUMB16_LSR_IMM, scratch, reg_armidx(dst), 9
            );
            frag_bfi(reg_armidx(dst), scratch, 8, 8);
            break;
        case SHIFT_RIGHT_L:
            frag_sbfx(scratch, reg_armidx(dst), 9, 7);
            frag_bfi(reg_armidx(dst), scratch, 8, 8);
            break;
        default:
            assert(false);
            break;
        }
    }
    else if (dst == OPARG_HLm)
    {
        frag_rotate_helper(rt, 1, 0, carry_in, carry_out, setz);
        frag_post_readwrite_hlm();
    }
    else
    {
        assert(false);
    }
}

static void frag_swap(sm83_oparg_t dst)
{
    // note: we could speed this up if we interlace the
    // nhc flag clearing into the main swap section;
    // this would 
    assert(is_reg8(dst) || dst == OPARG_HLm);
    
    if (dst == OPARG_A)
    {
        #if SETFLAGS
        // f: z=*
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, REG_nZ, REG_A
        );
        #endif
        frag_bfi(REG_A, REG_A, 8, 4);
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, REG_A, REG_A, 4
        );
    }
    else if (is_reglo(dst))
    {
        uint8_t altreg = 0;
        #if SETFLAGS
        altreg = REG_nZ;
        #endif
        
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, altreg, reg_armidx(dst), 4
        );
        
        frag_bfi(altreg, reg_armidx(dst), 4, 4);
        frag_bfi(reg_armidx(dst), altreg, 0, 8);
        
        #if SETFLAGS
        // z=*
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, REG_nZ, REG_nZ
        );
        #endif
    }
    else if (is_reghi(dst))
    {
        // TODO: optimize
        frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, 0, reg_armidx(dst), 4);
        
        frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_nZ, reg_armidx(dst), 4);
        
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, reg_armidx(dst), reg_armidx(dst)
        );
        
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM, REG_nZ, REG_nZ, false, 0xf0, 24
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, REG_nZ, 0
        );
        
        // zero-out low byte
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM, REG_nZ, REG_nZ, false, 0xff, 24
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, reg_armidx(dst), REG_nZ
        );
        
        // (we get z=* for free)
    }
    else if (dst == OPARG_HLm)
    {
        frag_pre_readwrite_hlm();
        
        // r0 now contains (HL)
        
        frag_bfi(0, 0, 8, 4);
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, 1, 0, 4
        );
        
        #if SETFLAGS
        // f: z=*
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, REG_nZ, 0
        );
        #endif
        
        frag_post_readwrite_hlm();
    }
    else
    {
        assert(false);
    }
    
    
    #if SETFLAGS
    // f: nhc = 0
    frag_ld_imm16(REG_Carry, 0);
        
    frag_set_nh(false, false);
    #endif
}

static void frag_edi(bool enable)
{
    dis.edi = enable ? 2 : 1;
    
    if (enable)
    {
        // we've enabled interrupts; it's possible that
        // the game wants one to trigger after the next
        // instruction, so we should stop now.
        dis.done = true;
    }
}

static inline uint16_t get_current_pc(void)
{
    return dis.pcstart + (dis.rom - dis.romstart);
}

static void frag_push_pc(void)
{
    const uint16_t pc = get_current_pc();
    const uint16_t regs = REGS_NONFLEX & 0xF;
    
    dis.cycles += 2;
    
    frag_add_imm16(0, REG_SP, -2);
    frag_armpush(regs);
    frag_uxth(REG_SP, 0);
    frag_uxth(0, 0);
    frag_ld_imm16(1, pc);
    frag_bl((fn_type)opts.writeword);
    frag_armpop(regs);
}

static inline void frag_condjump(sm83_oparg_t condition, bool istrue, uint8_t label)
{
    switch (condition)
    {
    case COND_none:
        return;
    case COND_c:
        frag_cmpbranch(true ^ !istrue, REG_Carry, label);
        break;
    case COND_nc:
        frag_cmpbranch(false ^ !istrue, REG_Carry, label);
        break;
    case COND_z:
        frag_cmpbranch(false ^ !istrue, REG_nZ, label);
        break;
    case COND_nz:
        frag_cmpbranch(true ^ !istrue, REG_nZ, label);
        break;
    default:
        assert(false);
        break;
    }
}

static void frag_jump(sm83_oparg_t condition, sm83_oparg_t addrmode)
{
    uint16_t immediate = 0;
    if (addrmode == ADDR_r8)
    {
        immediate = (int8_t)sm83_next_byte();
        immediate += get_current_pc();
    }
    else if (addrmode == ADDR_d16)
    {
        immediate = sm83_next_word();
    }
    else
    {
        assert(addrmode == ADDR_HL);
    }
    
    uint8_t condcycles = 0;
    if (condition != COND_none)
    {
        frag_condjump(condition, false, dis.next_label);
        condcycles = 3;
    }
    else
    {
        dis.cycles += 1;
    }
     
    if (addrmode == ADDR_HL)
    {
        frag_epilogue(-1, dis.cycles + condcycles);
    }
    else
    {
        frag_epilogue(immediate, dis.cycles + condcycles);
    }
    
    if (condition != COND_none)
    {
        frag_label(dis.next_label++);
    }
    else
    {
        dis.epilogue = 1;
    }
    
    dis.done = true;
}

static void frag_call(sm83_oparg_t condition)
{
    const uint16_t newpc = sm83_next_word();
    
    uint8_t condcycles = 0;
    if (condition != COND_none)
    {
        frag_condjump(condition, false, dis.next_label);
        condcycles = 3;
    }
    else
    {
        dis.cycles += 3;
    }
    
    frag_push_pc();
    frag_epilogue(newpc, dis.cycles + condcycles);
    
    if (condition != COND_none)
    {
        frag_label(dis.next_label++);
    }
    else
    {
        dis.epilogue = 1;
    }
    dis.done = true;
}

static void frag_rst(unsigned x)
{
    dis.cycles += 3;
    frag_push_pc();
    frag_epilogue(x*8, dis.cycles);
    dis.epilogue = 1;
    dis.done = true;
}

static void frag_ret(sm83_oparg_t condition)
{
    uint8_t condcycles = 0;
    if (condition != COND_none)
    {
        dis.cycles += 1;
        frag_condjump(condition, false, dis.next_label);
        condcycles = 3;
    }
    else
    {
        dis.cycles += 3;
    }
    
    frag_epilogue(-2, dis.cycles + condcycles);
    
    if (condition != COND_none)
    {
        frag_label(dis.next_label++);
    }
    else
    {
        dis.epilogue = true;
    }
    
     dis.done = true;
}

static void frag_reti(void)
{
    frag_edi(true);
    frag_ret(COND_none);
}

// convert to bcd
static void frag_daa(void)
{
    const uint8_t regs = (REGS_NONFLEX & ~(1 << REG_A)) & 0xe;
    frag_armpush(regs);
    
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_NH);
    assert(REG_A == 1);
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, 2, REG_Carry);
    
    frag_bl((fn_type)jit_regfile_daa);
    
    #if SETFLAGS
    // h <- 0
    frag_instr16_rd0_rm3(
        THUMB16_UXTB, REG_NH, REG_NH
    );
    
    // c <- r1
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_Carry, 1);
    #endif
    
    frag_armpop(regs);
    
    #if SETFLAGS
    // !z <- r0
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, 0);
    #endif
    
    // r%a <- r0
    frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_A, 0);
}

// set carry flag
static void frag_scf(void)
{
    #if SETFLAGS
    frag_ld_imm16(REG_Carry, 1);
    frag_set_nh(0, 0);
    #endif
}

// flip carry flag
static void frag_ccf(void)
{
    #if SETFLAGS
    frag_imm12c_rd8_rn16(
        THUMB32_EOR_IMM,
        REG_Carry, REG_Carry, false, 1, 0
    );
    frag_set_nh(0, 0);
    #endif
}

typedef enum
{
    AND,
    OR,
    XOR,
    ADD,
    ADC,
    CP,
    SUB,
    SBC,
} arithop_t;

static void frag_bitwise(sm83_oparg_t src, arithop_t arithop)
{
    dis.cycles += reg_cycles(src);
    
    uint16_t op16 = THUMB16_ORR_REG;
    uint32_t op32 = THUMB32_ORR_REG;
    uint32_t op32imm = THUMB32_ORR_IMM;
    
    if (arithop == AND)
    {
        op16 = THUMB16_AND_REG;
        op32 = THUMB32_AND_REG;
        op32imm = THUMB32_AND_IMM;
    }
    
    if (arithop == XOR)
    {
        op16 = THUMB16_EOR_REG;
        op32 = THUMB32_EOR_REG;
        op32imm = THUMB32_EOR_IMM;
    }
    
    if (src == OPARG_A)
    {
    src_a:
        switch(arithop)
        {
        case AND:
            #if SETFLAGS
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_A);
            #endif
            break;
            
        case OR:
            #if SETFLAGS
            frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_A);
            #endif
            break;
            
        case XOR:
            frag_ld_imm16(REG_A, 0);
            #if SETFLAGS
            frag_ld_imm16(REG_nZ, 0);
            #endif
            break;
            
        default:
            assert(false);
            break;
        }
    }
    else if (src == OPARG_i8)
    {
        const uint8_t immediate = sm83_next_byte();
        
        // shortcuts
        if (immediate == 0xff && arithop == AND) goto src_a;
        else if (immediate == 0x00 && arithop != AND)
        {
            arithop = OR;
            goto src_a;
        }
        
        // the main event
        frag_imm12c_rd8_rn16(
            op32imm, REG_A, REG_A, false, immediate, 0
        );
        
        #if SETFLAGS
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_A);
        #endif
    }
    else if (src == OPARG_HLm)
    {
        frag_read_hlm();
        
        frag_instr16_rd0_rm3(
            op16, REG_A, 0
        );
        
        #if SETFLAGS
        frag_ld_imm16(REG_nZ, REG_A);
        #endif
    }
    else if (is_reglo(src))
    {
        frag_instr16_rd0_rm3(
            op16, REG_A, reg_armidx(src)
        );
        
        #if SETFLAGS
        frag_instr16_rd0_rm3(
            (arithop == AND) ? THUMB16_UXTB : THUMB16_MOV_REG, REG_nZ, REG_A
        );
        #endif
        
        if (arithop != AND)
        {
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, REG_A, REG_A
            );
        }
    }
    else if (is_reghi(src))
    {
        frag_rd8_rn16_rm0_shift(
            op32, REG_A, REG_A, reg_armidx(src), IMM_SHIFT_LSR, 8
        );
        
        #if SETFLAGS
        frag_ld_imm16(REG_nZ, REG_A);
        #endif
    }
    else
    {
        assert(false);
    }
    
    #if SETFLAGS
    switch (arithop)
    {
    case AND:
        frag_set_nh(0, 1);
        break;
        
    case OR:
    case XOR:
        frag_set_nh(0, 0);
        break;
    
    default:
        assert(false);
        break;
    }
    frag_ld_imm16(REG_Carry, 0);
    #endif
}

static void frag_arithmetic(sm83_oparg_t dst, sm83_oparg_t src, arithop_t arithop)
{
    if (src == OPARG_i8 && dst == OPARG_SP && arithop == ADD)
    {
        // special case: add sp,i8
        frag_spi8(dst);
        return;
    }
    
    dis.cycles += reg_cycles(src);
    dis.cycles += reg_cycles(dst);
    
    const bool carry_in = (arithop == ADC || arithop == SBC);
    const bool subtract = (arithop == SUB || arithop == SBC || arithop == CP);
    if (carry_in)
    {
        // optimization: we could set the (arm) carry bit flag here to the value in REG_C
        // then we can use the adc/sbc operations.
    }
    
    #if SETFLAGS
    if (dst != REG_HL || src != REG_HL) // (we set nh directly in the add hl,hl case later)
    {
        // note: h flag will be modified further later.
        frag_set_nh(subtract, 0);
    }
    #else
    if (arithop == CP) return;
    #endif
    
    const uint8_t dstreg = (arithop == CP)
        ? REG_nZ
        : REG_A;
    
    if (dst == OPARG_A)
    {
        if (src == OPARG_i8)
        {
            uint8_t immediate = sm83_next_byte();
            // note: we can't replace with inc/dec if immediate is 1; flags are different.
                
            #if SETFLAGS
            // nh[3] <- REG_A
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 24
            );
            #endif
            
            frag_add_imm16(dstreg, REG_A,
                (subtract) ? -(int32_t)immediate : (int32_t)immediate
            );
            
            if (carry_in)
            {
                #if SETFLAGS
                // nh[1] <- carry in
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 8
                );
                #endif
                
                // r%dst += c
                frag_instr16_rd0_rn3_rm6(THUMB16_ADD_REG_T1, dstreg, REG_A, REG_Carry);
            }
            
            #if SETFLAGS
            // c <- carry out
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, dstreg, 8);
            
            // nh[2] <- imm
            frag_imm12c_rd8_rn16(
                THUMB32_ORR_IMM,
                REG_NH, REG_NH, false, immediate, 16
            );
            
            if (dstreg != REG_nZ)
            {
                // !z <- r%a
                frag_instr16_rd0_rm3(
                    THUMB16_UXTB, REG_nZ, dstreg
                );
            }
            #endif
            
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, dstreg, dstreg
            );
        }
        else if (src == OPARG_A)
        {
            // cannot replace with sla/rlca because flags are different
            switch (arithop)
            {
                case ADD:
                case ADC:
                    frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_A, REG_A, 1);
                    
                    #if SETFLAGS
                    // nh[2] <- a
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 16
                    );
                    // nh[3] <- a
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 24
                    );
                    #endif
                    
                    if (carry_in)
                    {
                        frag_instr16_rd0_rm3(
                            THUMB16_ORR_REG, REG_A, REG_Carry
                        );
                    }
                    
                    #if SETFLAGS
                    // h[1] <- c
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 8
                    );
                    
                    // !z <- r%a
                    frag_instr16_rd0_rm3(
                        THUMB16_UXTB, REG_nZ, REG_A
                    );
                    
                    // c <- r%a >> 8
                    frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, REG_A, 8);
                    #endif

                    frag_instr16_rd0_rm3(
                        THUMB16_UXTB, REG_A, REG_A
                    );
                    break;
                
                case SUB:
                    frag_ld_imm16(REG_A, 0);
                    #if SETFLAGS
                    frag_ld_imm16(REG_nZ, 1); // z <- 0
                    frag_ld_imm16(REG_Carry, 0);
                    //frag_set_nh(1, 0); // (already done above)
                    #endif
                    break;
                    
                case SBC:
                    frag_sbfx(REG_A, REG_Carry, 0, 1);
                    #if SETFLAGS
                    // n <- 0
                    frag_set_nh(1, 0);
                    // !z <- c
                    frag_instr16_rd0_rm3(THUMB16_MOV_REG, REG_nZ, REG_Carry);
                    // h <- c
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 12
                    );
                    // (c unmodified)
                    #endif
                    frag_instr16_rd0_rm3(
                        THUMB16_UXTB, REG_A, REG_A
                    );
                    break;
                
                case CP:
                    #if SETFLAGS
                    frag_ld_imm16(REG_nZ, 0); // z <- 1
                    frag_set_nh(1, 0);
                    frag_ld_imm16(REG_Carry, 0);
                    #endif
                    break;
                
                default:
                    assert(false);
                    break;
            }
        }
        else if (src == OPARG_HLm)
        {
            frag_read_hlm();
        
        operand_r0:
            switch (arithop)
            {
            case ADD:
            case ADC:
                #if SETFLAGS
                // nh[2] <- r0
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, 0, IMM_SHIFT_LSL, 16
                );
                
                // h[3] <- a
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 24
                );
                #endif
                
                // r%a += r0
                frag_instr16_rd0_rn3_rm6(THUMB16_ADD_REG_T1, REG_A, REG_A, 0);
                
                #if SETFLAGS
                if (carry_in)
                {
                    // h[1] <- carry
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 8
                    );
                }
                #endif
                
                if (carry_in)
                {
                    // r%a += r%carry
                    frag_instr16_rd0_rn3_rm6(THUMB16_ADD_REG_T1, REG_A, REG_A, 0);
                }
                
                #if SETFLAGS
                // c <- r%a >> 8
                frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, REG_A, 8);
                
                // !z <- r%a
                frag_instr16_rd0_rm3(
                    THUMB16_UXTB, REG_nZ, REG_A
                );
                #endif
                
                frag_instr16_rd0_rm3(
                    THUMB16_UXTB, REG_A, REG_A
                );
                break;
                
            case SUB:
            case SBC:
            case CP:
                #if SETFLAGS
                // h[2] <- r0
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, 0, IMM_SHIFT_LSL, 16
                );
                
                // h[3] <- r%a
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 24
                );
                #endif
            
                // r%dst <- r%a - r0
                frag_instr16_rd0_rn3_rm6(THUMB16_SUB_REG, dstreg, REG_A, 0);
                
                if (carry_in)
                {
                    #if SETFLAGS
                    // h[1] <- c
                    frag_rd8_rn16_rm0_shift(
                        THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 8
                    );
                    #endif
                    
                    // r%dst -= r%c
                    frag_instr16_rd0_rn3_rm6(THUMB16_SUB_REG, dstreg, dstreg, REG_Carry);
                }
                
                #if SETFLAGS
                // c <- r%dst.8
                frag_ubfx(REG_Carry, dstreg, 8, 1);
                
                if (dstreg != REG_nZ)
                {
                    // !z <- r%a
                    frag_instr16_rd0_rm3(
                        THUMB16_UXTB, REG_nZ, dstreg
                    );
                }
                #endif
                
                frag_instr16_rd0_rm3(
                    THUMB16_UXTB, dstreg, dstreg
                );
                break;
                
            default:
                assert(false);
                break;
            }
        }
        else if (is_reglo(src))
        {
            // r0 <- r%src & 0xff
            frag_instr16_rd0_rm3(
                THUMB16_UXTB, 0, reg_armidx(src)
            );
            
            // note: we could do better by swapping the order of set flags r0
            goto operand_r0;
        }
        else if (is_reghi(src))
        {
            const uint8_t reg = reg_armidx(src);
            
            // note: we could do better with add-shift
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, 0, reg_armidx(src), 8);
            
            goto operand_r0;
        }
        else
        {
            assert(false);
        }
    }
    else if (dst == REG_HL && is_reg16(src) && arithop == ADD)
    {
        // note: Z is not to be modified here.
        
        if (src == REG_HL)
        {
            #if SETFLAGS
            // c <- r%hl >> 15
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, REG_HL, 15);
            #endif
            
            // r%hl <<= 2
            frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_HL, REG_HL, 1);
            
            #if SETFLAGS
            // nh <- r%hl & $1000
            // (n <- 0)
            frag_imm12c_rd8_rn16(
                THUMB32_AND_IMM, REG_NH, REG_HL, false, 1, 20
            );
            #endif
            
            frag_instr16_rd0_rm3(
                THUMB16_UXTH, REG_HL, REG_HL
            );
        }
        else
        {
            // setting nh is quite curious here.
            
            #if SETFLAGS
            // r%nh <- r%hl ^ r%src
            frag_rd8_rn16_rm0_shift(
                THUMB32_EOR_REG, REG_NH, REG_HL, reg_armidx(src), 0, 0
            );
            #endif
            
            // r%hl += r%src
            frag_add_rd_rn(REG_HL, reg_armidx(src));
            
            #if SETFLAGS
            // r%nh ^= r%hl
            frag_instr16_rd0_rm3(
                THUMB16_UXTH, REG_NH, REG_HL
            );
            
            // c <- r%hl >> 16
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, REG_HL, 16);
            
            // nh[0,2,3] <- 0
            // nh[1] <- r%nh & 0x1000
            frag_imm12c_rd8_rn16(
                THUMB32_AND_IMM, REG_NH, REG_NH, false, 1, 20
            );
            #endif
            
            frag_instr16_rd0_rm3(
                THUMB16_UXTH, REG_HL, REG_HL
            );
        }
    }
    else
    {
        assert(false);
    }
}

static void frag_add(sm83_oparg_t dst, sm83_oparg_t src, int carry)
{
    frag_arithmetic(dst, src, carry ? ADC : ADD);
}

static void frag_sub(sm83_oparg_t dst, sm83_oparg_t src, int carry)
{
    frag_arithmetic(dst, src, carry ? SBC : SUB);
}

static void frag_cp(sm83_oparg_t cmp)
{
    frag_arithmetic(OPARG_A, cmp, CP);
}

static void frag_and(sm83_oparg_t src)
{
    frag_bitwise(src, AND);
}

static void frag_or(sm83_oparg_t src)
{
    frag_bitwise(src, OR);
}

static void frag_xor(sm83_oparg_t src)
{
    frag_bitwise(src, XOR);
}

static void frag_bit(unsigned b, sm83_oparg_t src)
{
    dis.cycles += reg_cycles(src);
    const uint8_t bit_offset = (is_reghi(src)) ? 8 : 0;
    uint8_t reg = reg_armidx(src);
    if (src == OPARG_HLm)
    {
        reg = 0;
        frag_read_hlm();
    }
    
    #ifdef SETFLAGS
    frag_set_nh(0, 1);
    frag_ubfx(REG_nZ, reg, b + bit_offset, 1);
    #endif
}

static void frag_push(sm83_oparg_t src)
{   
    if (src == OPARG_AF)
    {
        uint16_t regs = REGS_NONFLEX & 0xF;
        
        // subs r%sp, 2 [T2]
        frag_add_imm16(REG_SP, REG_SP, -2);
        
        frag_armpush(regs);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_Carry);
        // uxth r%sp, r%sp
        frag_uxth(REG_SP, REG_SP);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 1, REG_nZ);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 2, REG_NH);
        frag_bl((fn_type)jit_regfile_get_f);
        
        if ((1 << REG_A) & 0xF & REGS_NONFLEX)
        {
            // recover r1 <- A
            const unsigned int depth_A = __builtin_popcount(REGS_NONFLEX & 0xF & ((1 << REG_A) - 1));
            
            // ldr r1, [sp, depth_A * 4]
            frag_instr16_rd0_imm8(
                THUMB16_LDR_IMM_T2,
                1,
                depth_A
            );
            
            // ORR r1,r0,r1<<8
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, 1, 0, 1, IMM_SHIFT_LSL, 8
            );
        }
        else
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, 1, 0, REG_A, IMM_SHIFT_LSL, 8
            );
        }
        
        frag_mov_rd_rm(0, REG_SP);
        frag_bl((fn_type)opts.writeword);
        frag_armpop(regs);
    }
    else if (is_reg16(src))
    {
        uint16_t regs = REGS_NONFLEX & 0xF;
        
        frag_armpush(regs);
        
        // subs r%sp, 2 [T2]
        frag_add_imm16(REG_SP, REG_SP, -2);
        
        // mov r1, r%src
        frag_mov_rd_rm(1, reg_armidx(src));
        
        // uxth r%sp, r%sp
        frag_uxth(REG_SP, REG_SP);
        
        // mov r0, r%sp
        frag_mov_rd_rm(0, REG_SP);
        
        frag_bl((fn_type)opts.writeword);
        
        frag_armpop(regs);
    }
    else
    {
        assert(false);
    }
}

static void frag_pop(sm83_oparg_t dst)
{
    dis.cycles += 1;
    
    if (dst == OPARG_AF)
    {
        const uint16_t regs = (REGS_NONFLEX & ~(1 << REG_A)) & 0xf;
        frag_armpush(regs);
        
        frag_mov_rd_rm(0, REG_SP);
        
        frag_bl((fn_type)opts.readword);
        
        // nh <- r0 << 7
        // (shifts 'h' to bit 12)
        frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_NH, 0, 7);
        
        // a <- r0 >> 8
        frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_A, 0, 8);
        
        // adds r%sp, 2 [T2]
        frag_add_imm16(REG_SP, REG_SP, 2);
        
        // r0 <- r0 & 0xFF
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, 0, 0
        );
        
        // nh &= $1000
        // (masks to just bit 12)
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM,
            REG_NH, REG_NH, false, 1, 20
        );
        
        // c <- bit(r0, 4)
        frag_ubfx(REG_Carry, 0, 4, 1);
        
        // nh |= (r0 >> 1)
        // (shifts 'n' into bit 5; also some garbage in the lower byte)
        frag_rd8_rn16_rm0_shift(
            THUMB32_ORR_REG, REG_NH, REG_NH, 0, IMM_SHIFT_LSR, 1
        );
        
        // r%nz <- bit(r0, 7)
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM,
            REG_nZ, 0, false, 0x80, 0
        );
        
        // uxth r%sp, r%sp
        frag_uxth(REG_SP, REG_SP);
        
        // !z <- r%nz
        frag_imm12c_rd8_rn16(
            THUMB32_EOR_IMM,
            REG_nZ, REG_nZ, false, 0x80, 0
        );
        
        frag_armpop(regs);
    }
    else if (is_reg16(dst))
    {
        const uint16_t regs = (REGS_NONFLEX & ~(1 << reg_armidx(dst))) & 0xf;
        
        frag_armpush(regs);
        
        // mov r0, r%sp
        frag_mov_rd_rm(0, REG_SP);
        
        frag_bl((fn_type)opts.readword);
        
        // adds r%sp, 2 [T2]
        frag_add_imm16(REG_SP, REG_SP, 2);
        
        frag_armpop(regs);
        
        // uxth r%sp, r%sp
        frag_uxth(REG_SP, REG_SP);
        
        // mov r%dst, r0
        frag_mov_rd_rm(reg_armidx(dst), 0);
    }
    else
    {
        assert(false);
    }
}

static int disassemble_done(void)
{
    if (dis.error) return 1;
    if (dis.rom >= dis.romend - 2) return 1;
    if (dis.fragc >= 0x200) return 1;
    if (dis.next_label >= JIT_MAX_LABELS-1) return 1;
    
    // stop after enough sm83 instruction bytes, but prefer to stop
    // at a consistent spot.
    const uintptr_t sm83c = dis.rom - dis.initrom;
    if (sm83c >= 0x18 && sm83c % 0x10 == 0)
    {
        return 1;
    }
    if (sm83c >= 0x19 && sm83c % 0x10 == 1) // some sm83 instructions are two bytes; this catches that.
    {
        return 1;
    }
    
    return dis.done;
}

static void disassemble_begin(uint32_t gb_rom_offset, uint32_t gb_start_offset, uint32_t gb_end_offset)
{
    memset(&dis, 0, sizeof(dis));
    dis.fragcap = 0x100;
    dis.pcstart = opts.pc;
    dis.frag = (frag_t*)malloc(dis.fragcap * sizeof(frag_t));
    if (!dis.frag)
    {
        dis.error = 1;
        return;
    }
    for (size_t i = 0; i < JIT_START_PADDING_ENTRY_C; ++i)
    {
        dis.frag[i] = FRAG(
            .produce = fasm_skip
        );
    }
    dis.fragc = JIT_START_PADDING_ENTRY_C;
    dis.rom = (const uint8_t*)opts.rom + gb_rom_offset;
    dis.initrom = dis.rom;
    dis.romend = (const uint8_t*)opts.rom + gb_end_offset;
    dis.romstart = (const uint8_t*)opts.rom + gb_start_offset;
}

static void disassemble_capacity(void)
{
    if (dis.fragc >= dis.fragcap - 0x80)
    {
        dis.fragcap *= 2;
        frag_t* arm = (frag_t*)realloc(dis.frag, dis.fragcap * sizeof(frag_t));
        if (arm == NULL)
        {
            dis.error = 1;
        }
        else
        {
            dis.frag = arm;
        }
    }
}

static sm83_oparg_t arg_from_op(uint8_t op)
{
    switch (op % 0x8)
    {
        case 0:
            return OPARG_B;
        case 1:
            return OPARG_C;
        case 2:
            return OPARG_D;
        case 3:
            return OPARG_E;
        case 4:
            return OPARG_H;
        case 5:
            return OPARG_L;
        case 6:
            return OPARG_HLm;
        case 7:
            return OPARG_A;
    }
    
    // unused
    return OPARG_A;
}

static void collect_dependencies(void)
{
    #define IN(...)
    #define OUT(...)
    uint8_t op;
    switch (op = sm83_next_byte())
    {
        case 0x00: // NOP
            break;
            
        case 0x01:
            // ld BC, i16
            OUT(BC);
            break;
            
        case 0x02:
            // ld (BC), A
            IN(BC, A);
            break;
            
        case 0x03:
            // inc BC
            IN(BC);
            OUT(BC);
            break;
        
        case 0x04:
            // inc B
            IN(BC);
            OUT(BC, Z, NH);
            break;
            
        case 0x05:
            // dec B
            IN(BC);
            OUT(BC, Z, NH);
            break;
            
        case 0x06:
            // ld b, i8
            OUT(BC);
            break;
            
        case 0x07:
            // rlca
            IN(A, C);
            OUT(A, Z, NH, C);
            break;
            
        case 0x08:
            // ld (i16), SP
            IN(SP);
            break;
            
        case 0x09:
            // add HL, BC
            IN(HL, BC);
            OUT(HL, NH, C);
            return frag_add(OPARG_HL, OPARG_BC, 0);
            
        case 0x0A:
            // ld A, (BC)
            IN(BC);
            OUT(A);
            break;
            
        case 0x0B:
            // dec BC
            IN(BC);
            OUT(BC);
            break;
            
        case 0x0C:
            // inc c
            IN(BC);
            OUT(BC, Z, NH);
            break;
            
        case 0x0D:
            // dec c
            IN(BC);
            OUT(BC, Z, NH);
            break;
        
        case 0x0E:
            // ld C, i8
            OUT(BC);
            break;
            
        case 0x0F:
            // rrca
            IN(A, C);
            OUT(A, Z, NH, C);
            break;
        
        case 0x10: // STOP
            break;
            
        case 0x11:
            // ld de, i16
            OUT(DE)
            break;
            
        case 0x12:
            // ld (DE), A
            IN(DE, A);
            break;
            
        case 0x13:
            // inc DE
            IN(DE);
            OUT(DE);
            break;
            
        case 0x14:
            // inc D
            IN(DE);
            OUT(DE, Z, NH);
            break;
            
        case 0x15:
            // dec D
            IN(DE);
            OUT(DE, Z, NH);
            break;
            
        case 0x16:
            // ld D, i8
            OUT(DE);
            break;
            
        case 0x17:
            // rla
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0x18:
            // jr
            break;
            
        case 0x19:
            // add hl, de
            IN(HL, DE);
            OUT(HL, NH, C);
            break;
            
        case 0x1A:
            // ld A, (DE)
            IN(DE);
            OUT(A);
            break;
            
        case 0x1B:
            // dec DE
            IN(DE);
            OUT(DE);
            break;
            
        case 0x1C:
            // inc E
            IN(E);
            OUT(E, Z, NH);
            break;
            
        case 0x1D:
            // dec E
            IN(E);
            OUT(E, Z, NH);
            break;
            
        case 0x1E:
            // ld E, i8
            OUT(E);
            break;
            
        case 0x1F:
            // RRA
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0x20:
            // jr nz
            IN(Z);
            break;
            
        case 0x21:
            // ld HL, i16
            OUT(HL);
            break;
            
        case 0x22:
            // ld (HL+), A
            IN(A, HL);
            break;
            
        case 0x23:
            // inc HL
            IN(HL);
            OUT(HL);
            break;
            
        case 0x24:
            // inc H
            IN(H);
            OUT(H, Z, NH);
            break;
            
        case 0x25:
            // dec H
            IN(H);
            OUT(H, Z, NH);
            break;
            
        case 0x26:
            // ld H, i8
            IN(H);
            break;
            
        case 0x27:
            // daa
            IN(A, NH, C);
            OUT(A, Z, NH, C); // technically, only H is modified.
            break;
            
        case 0x28:
            // jr z
            IN(Z);
            break;
            
        case 0x29:
            // add HL, HL
            IN(HL);
            OUT(HL, NH, C);
            break;
            
        case 0x2A:
            // ld A, (HL+)
            OUT(A);
            IN(HL);
            break;
            
        case 0x2B:
            // dec HL
            IN(HL);
            OUT(HL);
            break;
            
        case 0x2C:
            // inc L
            IN(HL);
            OUT(HL, Z, NH);
            return frag_inc(OPARG_L);
            
        case 0x2D:
            // dec L
            IN(HL);
            OUT(HL, Z, NH);
            break;
            
        case 0x2E:
            // ld L, i8
            OUT(HL);
            break;
            
        case 0x2F:
            IN(A);
            OUT(A, NH);
            break;
            
        case 0x30:
            // jr nc
            IN(C);
            break;
            
        case 0x31:
            // ld SP, i16
            OUT(SP);
            break;
            
        case 0x32:
            // ld (HL-), A
            IN(HL, A);
            break;
            
        case 0x33:
            // inc SP
            IN(SP);
            OUT(SP);
            break;
            
        case 0x34:
            // inc (HL)
            IN(HL);
            OUT(Z, NH);
            break;
            
        case 0x35:
            // dec (HL)
            IN(HL);
            OUT(Z, NH);
            break;
            
        case 0x36:
            // ld HL, i8
            OUT(HL);
            break;
            
        case 0x37:
            // scf
            OUT(NH, C);
            break;
            
        case 0x38:
            // jr C
            IN(C);
            break;
            
        case 0x39:
            // add HL, SP
            IN(HL, SP);
            OUT(SP, NH, C);
            break;
            
        case 0x3A:
            // ld A, (HL-)
            IN(HL)
            OUT(A);
            break;
            
        case 0x3B:
            // dec SP
            IN(SP);
            OUT(SP);
            break;
            
        case 0x3C:
            // inc A
            IN(A);
            OUT(A, Z, NH);
            break;
            
        case 0x3D:
            // dec A
            IN(A);
            OUT(A, Z, NH);
            break;
            
        case 0x3E:
            // ld A, i8
            OUT(A);
            break;
            
        case 0x3F:
            // ccf
            IN(C);
            OUT(NH, C);
            break;
        
        // register transfer
        case 0x40 ... 0x75:
        case 0x77 ... 0x7F:
            {
                uint8_t opx = sm83_next_byte();
                switch(opx%8)
                {
                case 0:
                case 1:
                    IN(BC);
                    break;
                case 2:
                case 3:
                    IN(DE);
                    break;
                case 4:
                case 5:
                case 6:
                    IN(HL);
                    break;
                case 7:
                    IN(A);
                default:
                    break;
                }
                switch(opx/8)
                {
                case 0:
                case 1:
                    OUT(BC);
                    break;
                case 2:
                case 3:
                    OUT(DE);
                    break;
                case 4:
                case 5:
                    OUT(HL);
                    break;
                case 7:
                    OUT(A);
                    break;
                
                default:
                    break;
                }
            }
            break;
            
        case 0x76: // HALT
            break;
            
        // arithmetic
        case 0x80 ... 0xBF:
            {
                OUT(A, Z, NH, C);
                const uint8_t opx = sm83_next_byte();
                switch (opx % 8)
                {
                case 0:
                case 1:
                    IN(BC);
                    break;
                case 2:
                case 3:
                    IN(DE);
                    break;
                case 4:
                case 5:
                case 6:
                    IN(HL);
                    break;
                case 7:
                    // ADD A takes A as input
                    // but SUB A effectively does not (result is always the same).
                    if (opx == 0x87 || opx == 0x8F || opx == 0xA7 || opx == 0xB7)
                    {
                        IN(A);
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        
        case 0xC0:
            // ret nz
            // we do not consider SP because it appears specially in the epilogue.
            IN(Z);
            break;
        
        case 0xC1:
            // pop bc
            IN(SP);
            OUT(SP, BC);
            break;
            
        case 0xC2:
            // jmp nz
            IN(Z);
            break;
            
        case 0xC3:
            // jmp
            break;
            
        case 0xC4:
            // call nz
            IN(SP, Z);
            OUT(SP);
            break;
            
        case 0xC5:
            // push BC
            IN(SP, BC);
            OUT(SP);
            break;
            
        case 0xC6:
            // add A, i8
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0xC7:
            // rst 0
            IN(SP);
            OUT(SP);
            break;
        
        case 0xC8:
            // ret z
            IN(Z);
            // SP not considered -- see above
            break;
        
        case 0xC9:
            // ret
            // SP not considered -- see above
            break;
            
        case 0xCA:
            // jmp z
            IN(Z);
            break;
            
        case 0xCB:
            {
                const bool bit = (op >= 0x40 && op < 0x80);
                const uint8_t opx = sm83_next_byte();
                switch(opx%8)
                {
                case 0:
                case 1:
                    IN(BC);
                    if (!bit) OUT(BC);
                    break;
                case 2:
                case 3:
                    IN(DE);
                    if (!bit) OUT(DE);
                    break;
                case 4:
                case 5:
                    IN(HL);
                    if (!bit) OUT(HL);
                    break;
                case 6:
                    IN(HL);
                    break;
                case 7:
                    IN(A);
                    if (!bit) OUT(A);
                }
                
                if (opx < 0x80)
                {
                    OUT(Z, NH);
                    if (!bit) OUT(C);
                    if (opx >= 0x10 && opx < 0x20)
                    {
                        IN(C);
                    }
                }
            }
            break;
            
        case 0xCC:
            // call Z
            IN(SP, Z);
            OUT(SP);
            break;
            
        case 0xCD:
            // call
            IN(SP);
            OUT(SP);
            break;
            
        case 0xCE:
            // adc a, i8
            IN(A, C);
            OUT(A, Z, NH, C);
            break;
            
        case 0xCF:
            // rst 1
            IN(SP);
            OUT(SP);
            break;
            
        case 0xD0:
            // ret nc
            IN(C);
            break;
        
        case 0xD1:
            // pop DE
            IN(SP);
            OUT(DE, SP);
            break;
            
        case 0xD2:
            // jp nc
            IN(C);
            break;
            
        case 0xD4:
            // call nc
            IN(C, SP);
            OUT(SP);
            break;
            
        case 0xD5:
            // push de
            return frag_push(OPARG_DE);
            
        case 0xD6:
            // sub a, i8
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0xD7:
            // rst 2
            IN(SP);
            OUT(SP);
            break;
        
        case 0xD8:
            // ret c
            IN(C);
            break;
        
        case 0xD9:
            // reti
            break;
            
        case 0xDA:
            // jmp c
            IN(C);
            break;
            
        case 0xDC:
            // call c
            IN(C, SP);
            OUT(SP);
            break;
            
        case 0xDE:
            // sbc A, i8
            IN(A, C);
            OUT(A, Z, NH, C);
            break;
            
        case 0xDF:
            // rst 3
            IN(SP);
            OUT(SP)
            break;
            
        case 0xE0:
            // ld (i8), A
            IN(A);
            break;
            
        case 0xE1:
            // pop HL
            IN(SP);
            OUT(SP, HL);
            break;
            
        case 0xE2:
            // ld (C), A
            IN(A, BC);
            break;
            
        case 0xE5:
            OUT(HL);
            break;
            
        case 0xE6:
            // and i8
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0xE7:
            // rst 4
            IN(SP);
            OUT(SP);
            break;
            
        case 0xE8:
            // add SP, i8
            IN(SP);
            OUT(SP);
            break;
            
        case 0xE9:
            // jmp HL
            IN(HL);
            break;
            
        case 0xEA:
            // ld (i16), A
            IN(A);
            break;
            
        case 0xEE:
            // xor i8
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0xEF:
            // rst 5
            IN(SP);
            OUT(SP);
            break;
            
        case 0xF0:
            // ld A, (i8)
            OUT(A);
            break;
            
        case 0xF1:
            // pop AF
            IN(SP);
            OUT(A, SP);
            
        case 0xF2:
            // ld A, (C)
            IN(C);
            OUT(A);
            break;
            
        case 0xF3:
            // di
            break;
            
        case 0xF5:
            // push AF
            IN(A, SP);
            OUT(SP);
            break;
            
        case 0xF6:
            // or i8
            IN(A);
            OUT(A, Z, NH, C);
            break;
            
        case 0xF7:
            // rst 6
            IN(PC);
            OUT(pC);
            break;
            
        case 0xF8:
            // ld HL, SP+i8
            IN(SP);
            OUT(HL);
            break;
            
        case 0xF9:
            // ld SP, HL
            IN(HL);
            OUT(SP);
            break;
            
        case 0xFA:
            // ld A, (i16)
            OUT(A);
            break;
            
        case 0xFB:
            // ei
            break;
            
        case 0xFE:
            // cp i8
            IN(A);
            OUT(Z, NH, C);
            break;
            
        case 0xFF:
            // rst 7
            IN(SP);
            OUT(SP);
            break;
            
        default:
            break;
    }
}

static void disassemble_instruction(void)
{
    disassemble_capacity();
    uint8_t op;
    switch (op = sm83_next_byte())
    {
        case 0x00: // NOP
            return;
            
        case 0x01:
            return frag_ld(OPARG_BC, OPARG_i16);
            
        case 0x02:
            return frag_ld(OPARG_BCm, OPARG_A);
            
        case 0x03:
            return frag_inc(OPARG_BC);
        
        case 0x04:
            return frag_inc(OPARG_B);
            
        case 0x05:
            return frag_dec(OPARG_B);
            
        case 0x06:
            return frag_ld(OPARG_B, OPARG_i8);
            
        case 0x07:
            return frag_rotate(ROT_LEFT_CIRCULAR, OPARG_A, false);
            
        case 0x08:
            return frag_ld(OPARG_i16m, OPARG_SP);
            
        case 0x09:
            return frag_add(OPARG_HL, OPARG_BC, 0);
            
        case 0x0A:
            return frag_ld(OPARG_A, OPARG_BCm);
            
        case 0x0B:
            return frag_dec(OPARG_BC);
            
        case 0x0C:
            return frag_inc(OPARG_C);
            
        case 0x0D:
            return frag_dec(OPARG_C);
        
        case 0x0E:
            return frag_ld(OPARG_C, OPARG_i8);
            
        case 0x0F:
            return frag_rotate(ROT_RIGHT_CIRCULAR, OPARG_A, false);
        
        case 0x10: // STOP
            dis.done = 1;
            return frag_delegate(opts.stop);
            
        case 0x11:
            return frag_ld(OPARG_DE, OPARG_i16);
            
        case 0x12:
            return frag_ld(OPARG_DEm, OPARG_A);
            
        case 0x13:
            return frag_inc(OPARG_DE);
            
        case 0x14:
            return frag_inc(OPARG_D);
            
        case 0x15:
            return frag_dec(OPARG_D);
            
        case 0x16:
            return frag_ld(OPARG_D, OPARG_i8);
            
        case 0x17:
            return frag_rotate(ROT_LEFT, OPARG_A, false);
            
        case 0x18:
            return frag_jump(COND_none, ADDR_r8);
            
        case 0x19:
            return frag_add(OPARG_HL, OPARG_DE, 0);
            
        case 0x1A:
            return frag_ld(OPARG_A, OPARG_DEm);
            
        case 0x1B:
            return frag_dec(OPARG_DE);
            
        case 0x1C:
            return frag_inc(OPARG_E);
            
        case 0x1D:
            return frag_dec(OPARG_E);
            
        case 0x1E:
            return frag_ld(OPARG_E, OPARG_i8);
            
        case 0x1F:
            return frag_rotate(ROT_RIGHT, OPARG_A, false);
            
        case 0x20:
            return frag_jump(COND_nz, ADDR_r8);
            
        case 0x21:
            return frag_ld(OPARG_HL, OPARG_i16);
            
        case 0x22:
            return frag_ld(OPARG_HLmi, OPARG_A);
            
        case 0x23:
            return frag_inc(OPARG_HL);
            
        case 0x24:
            return frag_inc(OPARG_H);
            
        case 0x25:
            return frag_dec(OPARG_H);
            
        case 0x26:
            return frag_ld(OPARG_H, OPARG_i8);
            
        case 0x27:
            return frag_daa();
            
        case 0x28:
            return frag_jump(COND_z, ADDR_r8);
            
        case 0x29:
            return frag_add(OPARG_HL, OPARG_HL, 0);
            
        case 0x2A:
            return frag_ld(OPARG_A, OPARG_HLmi);
            
        case 0x2B:
            return frag_dec(OPARG_HL);
            
        case 0x2C:
            return frag_inc(OPARG_L);
            
        case 0x2D:
            return frag_dec(OPARG_L);
            
        case 0x2E:
            return frag_ld(OPARG_L, OPARG_i8);
            
        case 0x2F:
            return frag_cpl();
            
        case 0x30:
            return frag_jump(COND_nc, ADDR_r8);
            
        case 0x31:
            return frag_ld(OPARG_SP, OPARG_i16);
            
        case 0x32:
            return frag_ld(OPARG_HLmd, OPARG_A);
            
        case 0x33:
            return frag_inc(OPARG_SP);
            
        case 0x34:
            return frag_inc(OPARG_HLm);
            
        case 0x35:
            return frag_dec(OPARG_HLm);
            
        case 0x36:
            return frag_ld(OPARG_HL, OPARG_i8);
            
        case 0x37:
            return frag_scf();
            
        case 0x38:
            return frag_jump(COND_c, ADDR_r8);
            
        case 0x39:
            return frag_add(OPARG_HL, OPARG_SP, 0);
            
        case 0x3A:
            return frag_ld(OPARG_A, OPARG_HLmd);
            
        case 0x3B:
            return frag_dec(OPARG_SP);
            
        case 0x3C:
            return frag_inc(OPARG_A);
            
        case 0x3D:
            return frag_dec(OPARG_A);
            
        case 0x3E:
            return frag_ld(OPARG_A, OPARG_i8);
            
        case 0x3F:
            return frag_ccf();
        
        // register transfer
        case 0x40 ... 0x75:
        case 0x77 ... 0x7F:
            switch (op / 0x8)
            {
                case 0x40/8:
                    return frag_ld(OPARG_B, arg_from_op(op));
                case 0x48/8:
                    return frag_ld(OPARG_C, arg_from_op(op));
                case 0x50/8:
                    return frag_ld(OPARG_D, arg_from_op(op));
                case 0x58/8:
                    return frag_ld(OPARG_E, arg_from_op(op));
                case 0x60/8:
                    return frag_ld(OPARG_H, arg_from_op(op));
                case 0x68/8:
                    return frag_ld(OPARG_L, arg_from_op(op));
                case 0x70/8:
                    return frag_ld(OPARG_HLm, arg_from_op(op));
                case 0x78/8:
                    return frag_ld(OPARG_A, arg_from_op(op));
            }
            
        case 0x76: // HALT
            dis.done = 1;
            JIT_DEBUG_MESSAGE("disassembling HALT");
            return frag_delegate(opts.halt);
            
        // arithmetic
        case 0x80 ... 0xBF:
            switch (op)
            {
            case 0x80 ... 0x87:
                return frag_add(OPARG_A, arg_from_op(op), 0);
            case 0x88 ... 0x8F:
                return frag_add(OPARG_A, arg_from_op(op), 1);
            case 0x90 ... 0x97:
                return frag_sub(OPARG_A, arg_from_op(op), 0);
            case 0x98 ... 0x9F:
                return frag_sub(OPARG_A, arg_from_op(op), 1);
            case 0xA0 ... 0xA7:
                return frag_and(arg_from_op(op));
            case 0xA8 ... 0xAF:
                return frag_xor(arg_from_op(op));
            case 0xB0 ... 0xB7:
                return frag_or(arg_from_op(op));
            case 0xB8 ... 0xBF:
                return frag_cp(arg_from_op(op));
            }
        
        case 0xC0:
            return frag_ret(COND_nz);
        
        case 0xC1:
            return frag_pop(OPARG_BC);
            
        case 0xC2:
            return frag_jump(COND_nz, ADDR_d16);
            
        case 0xC3:
            return frag_jump(COND_none, ADDR_d16);
            
        case 0xC4:
            return frag_call(COND_nz);
            
        case 0xC5:
            return frag_push(OPARG_BC);
            
        case 0xC6:
            return frag_add(OPARG_A, OPARG_i8, 0);
            
        case 0xC7:
            return frag_rst(0);
        
        case 0xC8:
            return frag_ret(COND_z);
        
        case 0xC9:
            return frag_ret(COND_none);
            
        case 0xCA:
            return frag_jump(COND_z, ADDR_d16);
            
        case 0xCB:
            {
                uint8_t opx = sm83_next_byte();
                sm83_oparg_t arg = arg_from_op(opx);
                switch(opx/8)
                {
                case 0x00/8:
                    return frag_rotate(ROT_LEFT_CIRCULAR, arg, true);
                case 0x08/8:
                    return frag_rotate(ROT_RIGHT_CIRCULAR, arg, true);
                case 0x10/8:
                    return frag_rotate(ROT_LEFT, arg, true);
                case 0x18/8:
                    return frag_rotate(ROT_RIGHT, arg, true);
                case 0x20/8:
                    return frag_rotate(SHIFT_LEFT, arg, true);
                case 0x28/8:
                    return frag_rotate(SHIFT_RIGHT, arg, true);
                case 0x30/8:
                    return frag_swap(arg);
                case 0x38/8:
                    return frag_rotate(SHIFT_RIGHT_L, arg, true);
                case 0x40/8 ... 0x78/8:
                    return frag_bit((opx/8 - 0x40/8), arg);
                case 0x80/8 ... 0xB8/8:
                    return frag_set((opx/8 - 0x80/8), arg, 0);
                case 0xC0/8 ... 0xF8/8:
                    return frag_set((opx/8 - 0xC0/8), arg, 1);
                }
            }
            
        case 0xCC:
            return frag_call(COND_z);
            
        case 0xCD:
            return frag_call(COND_none);
            
        case 0xCE:
            return frag_add(OPARG_A, OPARG_i8, 1);
            
        case 0xCF:
            return frag_rst(1);
            
        case 0xD0:
            return frag_ret(COND_nc);
        
        case 0xD1:
            return frag_pop(OPARG_DE);
            
        case 0xD2:
            return frag_jump(COND_nc, ADDR_d16);
            
        case 0xD4:
            return frag_call(COND_nc);
            
        case 0xD5:
            return frag_push(OPARG_DE);
            
        case 0xD6:
            return frag_sub(OPARG_A, OPARG_i8, 0);
            
        case 0xD7:
            return frag_rst(2);
        
        case 0xD8:
            return frag_ret(COND_c);
        
        case 0xD9:
            return frag_reti();
            
        case 0xDA:
            return frag_jump(COND_c, ADDR_d16);
            
        case 0xDC:
            return frag_call(COND_c);
            
        case 0xDE:
            return frag_sub(OPARG_A, OPARG_i8, 1);
            
        case 0xDF:
            return frag_rst(3);
            
        case 0xE0:
            return frag_ld(OPARG_i8m, OPARG_A);
            
        case 0xE1:
            return frag_pop(OPARG_HL);
            
        case 0xE2:
            return frag_ld(OPARG_Cm, OPARG_A);
            
        case 0xE5:
            return frag_push(OPARG_HL);
            
        case 0xE6:
            return frag_and(OPARG_i8);
            
        case 0xE7:
            return frag_rst(4);
            
        case 0xE8:
            return frag_spi8(OPARG_SP);
            
        case 0xE9:
            return frag_jump(COND_none, ADDR_HL);
            
        case 0xEA:
            return frag_ld(OPARG_i16m, OPARG_A);
            
        case 0xEE:
            return frag_xor(OPARG_i8);
            
        case 0xEF:
            return frag_rst(5);
            
        case 0xF0:
            return frag_ld(OPARG_A, OPARG_i8m);
            
        case 0xF1:
            return frag_pop(OPARG_AF);
            
        case 0xF2:
            return frag_ld(OPARG_A, OPARG_Cm);
            
        case 0xF3:
            return frag_edi(0);
            
        case 0xF5:
            return frag_push(OPARG_AF);
            
        case 0xF6:
            return frag_or(OPARG_i8);
            
        case 0xF7:
            return frag_rst(6);
            
        case 0xF8:
            return frag_spi8(OPARG_HL);
            
        case 0xF9:
            return frag_ld(OPARG_SP, OPARG_HL);
            
        case 0xFA:
            return frag_ld(OPARG_A, OPARG_i16m);
            
        case 0xFB:
            return frag_edi(1);
            
        case 0xFE:
            return frag_cp(OPARG_i8);
            
        case 0xFF:
            return frag_rst(7);
            
        default:
            dis.done = 1;
            frag_delegate(opts.illegal);
    }
}

static void prologue(void)
{
    // add some extra code at the start/end of a jit block
    // this will do things like e.g. ensure call convention correctness,
    // push/pop used registers, and load gb regs into/from arm regs.
    
    // epilogue_x: adds code to end of block.
    // prologue_x: adds code to start of block.
    
    // prologue size cannot exceed JIT_START_PADDING_ENTRY_C
    
    const unsigned fragc = dis.fragc;
    dis.fragc = 0;
    
    // we have to push r4-r12 if we use them -- they are callee-push registers.
    uint16_t reg_push = (REGS_ALL & ~0xf);
    if (true) //(dis.use_lr)
    {
        reg_push |= (1 << 14);
    }
    frag_armpush(reg_push);
    
    // [A6.7.40] LDM r0, { ... }
    frag_access_multiple(false, REGS_SM83, 0, false);
    
    // push { r0 }
    frag_armpush(1 << 0);
    
    JIT_DEBUG_MESSAGE("padding %d %d", dis.fragc, JIT_START_PADDING_ENTRY_C);
    assert(dis.fragc <= JIT_START_PADDING_ENTRY_C);
    dis.fragc = fragc;
}

static void* disassemble_end(void)
{
    if (dis.error)
    {
        if (dis.frag) free(dis.frag);
        return NULL;
    }
    
    if (!dis.epilogue)
    {
        frag_epilogue(get_current_pc(), dis.cycles);
    }
    
    prologue();
    
    // accumulate size of output
    size_t outsize = 0;
    for (size_t i = 0; i < dis.fragc; ++i)
    {
        uint8_t size = dis.frag[i].produce(dis.frag[i].args, NULL);
        #ifdef JIT_DEBUG
        dis.frag[i].length = size;
        #endif
        outsize += size;
    }
    
    // allocate output buffer
    uint16_t* const out = (uint16_t*)malloc(outsize * sizeof(uint16_t));
    if (!out)
    {
        dis.error = 1;
        if (dis.frag) free(dis.frag);
        return NULL;
    }
    
    JIT_DEBUG_MESSAGE("base address: %8p; gb addr: %04x", out, (uintptr_t)(dis.initrom - dis.romstart));
    uint16_t* outb = out;
    
    for (size_t i = 0; i < dis.fragc; ++i)
    {
        outb += dis.frag[i].produce(dis.frag[i].args, outb);
    }
    
    #ifdef JIT_DEBUG
    #ifndef TARGET_QEMU
    #endif
    //JIT_DEBUG_MESSAGE("arm code: ");
    const char* outmsg;
    size_t fragi = 0;
    size_t armseek = 0;
    for (const uint16_t* arm = out; arm && arm < out+outsize;)
    {
        // zip along with arm disp as well
        while (armseek < arm - out || (dis.frag[fragi].length == 0 && armseek < outsize))
        {
            armseek += dis.frag[fragi++].length;
        }
        
        if (armseek == arm - out)
        {
            if (dis.frag[fragi].romsrc)
            {
                sm38d(dis.frag[fragi].romsrc, &outmsg);
                JIT_DEBUG_MESSAGE("\n; %s", outmsg);
            }
        }
        const uint16_t* armprev = arm;
        arm = armd(arm, &outmsg);
        if (arm && arm == armprev+1)
        {
            JIT_DEBUG_MESSAGE("%04x ; %s", *armprev, outmsg);
        }
        else if (arm && arm == armprev+2)
        {
            JIT_DEBUG_MESSAGE("%04x%04x ; %s", armprev[0], armprev[1], outmsg);
        }
        else
        {
            JIT_DEBUG_MESSAGE("DISASM ERROR");
        }
    }
    JIT_DEBUG_MESSAGE(" Done.\n");
    spin(); spin(); spin();
    #endif
    
    free(dis.frag);
    
    return arm_interworking_thumb(out);
}

static jit_fn jit_compile(uint16_t gb_addr, uint16_t gb_bank)
{
    if (gb_addr >= 0x8000) return NULL;
    if (gb_addr == 0x7FFF || gb_addr == 0x3FFF || gb_addr == 0x7FFE || gb_addr == 0x3FFE) return NULL;
    
    uint32_t gb_rom_offset = (gb_addr % 0x4000) | ((uint32_t)gb_bank * 0x4000);
    uint32_t gb_start_offset = (((uint32_t)gb_bank) * 0x4000);
    uint32_t gb_end_offset = (((uint32_t)gb_bank + 1) * 0x4000);
    
    disassemble_begin(gb_rom_offset, gb_start_offset, gb_end_offset);
    
    while (!disassemble_done())
    // disassemble each gameboy instruction one at a time.
    {
        #ifdef JIT_DEBUG
        size_t armi_prev = dis.fragc;
        const uint8_t* romsrc = dis.rom;
        #endif
        
        disassemble_instruction();
        
        #ifdef JIT_DEBUG
        if (dis.fragc > armi_prev)
        {
            dis.frag[armi_prev].romsrc = romsrc;
        }
        #endif
    }
    
    jit_fn fn = (jit_fn)disassemble_end();
    if (!fn) return NULL;
    
    jit_add_ht_entry(gb_addr, gb_bank, fn);
    
    return (jit_fn)arm_interworking_thumb(fn);
}

jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank)
{
    jit_fn f = get_jit_fn(gb_addr, gb_bank);
    if (f) return f;
    
    // no existing fn -- create one.
    jit_fn fn = jit_compile(gb_addr, gb_bank);
    if (!fn) return NULL;
    
    #ifndef TARGET_QEMU
    opts.playdate->system->clearICache();
    #endif
    
    return fn;
}

