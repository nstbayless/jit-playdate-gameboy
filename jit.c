#include "jit.h"
#include "armd.h"
#include "sm38d.h"

/*
    Useful resources when developing this JIT:
    - Arm v7-m architecture reference manual: 
        - https://developer.arm.com/documentation/ddi0489/b/
        - This will help convert between arm opcodes and hexadecimal.
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
            - r5 = SP (high 2 bytes are always 0)
            - r6 = Carry (only 1 bit is used)
            - r7 = pointer to jit_regfile_t
            - (r0 is used as flex/scratch, and to return cycle count at the end)
            - (note that some registers might not be loaded from jit_regfile_t in the prologue if they are not inputs for this chunk)
            - (similarly, some registers might not be written to jit_regfile_t in the epilogue if they are not modified this chunk)
        - dynamic recompiling is done by translating in the following passes:
            1. Fragmentation: each sm83 instruction is converted into multiple "translation fragments," or just "fragments" for short (frag_t).
                - a fragment (frag_t) is a representation of an arm operation.
                - for example, it could represent a single 16-bit thumb instruction, or it could be "call a function at a particular address" (which takes several instructions)
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
            - often we push r1 when we don't need to (used_registers() | 2)
*/

/*
    OPTIMIZATION IDEAS
    - these sm83 commands don't exist: ld bc,hl; ld de,bc; and so on.
        - they are emulated in sm83 by instruction pairs e.g. push hl, pop bc; ld b,h; ld c, l
        - we can identify this and reduce to just one arm mov instruction.
        
    - we can analyze the rst routines and inline them
*/

#ifndef JIT_DEBUG
    #define JIT_DEBUG
#endif

#ifdef TARGET_PLAYDATE
    #ifdef __FPU_USED
        #undef __FPU_USED
    #endif
    #define STM32F746xx
    #include "stm32f7xx.h" 
    #define assert(_B) jit_assert_pd(_B, opts.playdate)
#else
    #include <assert.h>
#endif

#ifndef false
    #define false 0
#endif
#ifndef true
    #define true 1
#endif
typedef int bool;

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#define JIT_HASHTABLE_SIZE 0x4000 // must be power of 2, at most 0x8000
#define JIT_START_PADDING_ENTRY_C 10

#ifdef JIT_DEBUG
#define JIT_DEBUG_MESSAGE(...) do {opts.playdate->system->logToConsole(__VA_ARGS__); spin();} while(0)
#else
#define JIT_DEBUG_MESSAGE(...)
#endif

// TODO: assign registers programatically
// reg_flex MUST be reg 0, because r0 is the return register.
#define REG_REGF 7
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

typedef enum
{
    SHIFT_LEFT,
    ROT_LEFT,
    ROT_LEFT_CARRY,
    SHIFT_RIGHT,
    SHIFT_RIGHT_L,
    ROT_RIGHT,
    ROT_RIGHT_CARRY,
} rotate_type;

static jit_opts opts;
static jit_ht_entry* jit_hashtable[JIT_HASHTABLE_SIZE];

static void* arm_interworking_none(void* addr)
{
	#ifdef TARGET_PLAYDATE
	return (void*)((uintptr_t)addr & ~1);
	#else
	return addr;
	#endif
}

static void* arm_interworking_thumb(void* fn)
{
    #if TARGET_PLAYDATE
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
    for (const jit_ht_entry* p = prev; p && p->fn; ++p)
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
    if (prev)
    {
        free(prev);
    }
}

// gets jit fn if it exists, otherwise returns NULL.
static jit_fn get_jit_fn(uint16_t gb_addr, uint16_t gb_bank)
{
    uint16_t hash_entry = (gb_addr) % JIT_HASHTABLE_SIZE;
    jit_ht_entry* e = jit_hashtable[hash_entry];
    
    if (!e) return NULL;
    
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
    memset(jit_hashtable, 0, sizeof(jit_hashtable));
    //JIT_DEBUG_MESSAGE("memfix.\n");
    jit_memfix();
}

void jit_cleanup(void)
{
    for (size_t i = 0; i < JIT_HASHTABLE_SIZE; ++i)
    {
        if (jit_hashtable[i])
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
    uint16_t fragc;
    uint16_t fragcap;
    const uint8_t* romstart;
    const uint8_t* rom;
    const uint8_t* romend;

    unsigned done : 1;
    unsigned error : 1;
    
    // registers in, registers used, and registers out (i.e. modified)
    // if used: must push register if >= 4
    // if input: must read the register at start of block
    // if dirty: must write the register back to the regfile at the end of the block
    unsigned use_r_a: 1;
    unsigned input_r_a : 1;
    unsigned dirty_r_a : 1;
    unsigned use_r_bc: 1;
    unsigned input_r_bc : 1;
    unsigned dirty_r_bc : 1;
    unsigned use_r_de: 1;
    unsigned input_r_de : 1;
    unsigned dirty_r_de : 1;
    unsigned use_r_hl: 1;
    unsigned input_r_hl : 1;
    unsigned dirty_r_hl : 1;
    unsigned use_r_sp: 1;
    unsigned input_r_sp : 1;
    unsigned dirty_r_sp : 1;
    unsigned use_r_regfile : 1;
    unsigned use_r_flex : 1;
    unsigned use_lr : 1;
} dis;

static uint8_t sm83_next_byte(void)
{
    return *(dis.rom++);
}

static uint16_t sm83_next_word(void)
{
    uint16_t x = sm83_next_byte();
    x |= sm83_next_byte() << 8;
    return x;
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

// reg must be one of OPARG_A, OPARG_BC, OPARG_DE, OPARG_HL.
static void use_register(sm83_oparg_t reg, int dependency, int dirty)
{
    switch (reg)
    {
        case OPARG_AF:
            // fallthrough
        case OPARG_A:
            if (dependency && !dis.use_r_a)
            {
                dis.input_r_a = 1;
            }
            if (dirty)
            {
                dis.dirty_r_a = 1;
            }
            dis.use_r_a = 1;
            dis.use_r_regfile = 1;
            break;
        case OPARG_B:
        case OPARG_C:
        case OPARG_BC:
            if (dependency && !dis.use_r_bc)
            {
                dis.input_r_bc = 1;
            }
            if (dirty)
            {
                dis.dirty_r_bc = 1;
            }
            dis.use_r_bc = 1;
            dis.use_r_regfile = 1;
            break;
        case OPARG_D:
        case OPARG_E:
        case OPARG_DE:
            if (dependency && !dis.use_r_de)
            {
                dis.input_r_de = 1;
            }
            if (dirty)
            {
                dis.dirty_r_de = 1;
            }
            dis.use_r_de = 1;
            dis.use_r_regfile = 1;
            break;
        case OPARG_H:
        case OPARG_L:
        case OPARG_HL:
            if (dependency && !dis.use_r_hl)
            {
                dis.input_r_hl = 1;
            }
            if (dirty)
            {
                dis.dirty_r_hl = 1;
            }
            dis.use_r_hl = 1;
            dis.use_r_regfile = 1;
            break;
        case OPARG_SP:
            if (dependency && !dis.use_r_sp)
            {
                dis.input_r_sp = 1;
            }
            if (dirty)
            {
                dis.dirty_r_sp = 1;
            }
            dis.use_r_sp = 1;
            dis.use_r_regfile = 1;
            break;
        default:
            break;
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

static void frag_instr32(uint32_t instr)
{
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)(uintptr_t)instr,
        .produce = fasm_32
    );
}

static uint8_t input_registers(void)
{
    uint32_t reg_push = 0;
    if (dis.input_r_a)
    {
        reg_push |= 1 << REG_A;
    }
    if (dis.input_r_bc)
    {
        reg_push |= 1 << REG_BC;
    }
    if (dis.input_r_de)
    {
        reg_push |= 1 << REG_DE;
    }
    if (dis.input_r_hl)
    {
        reg_push |= 1 << REG_HL;
    }
    if (dis.input_r_sp)
    {
        reg_push |= 1 << REG_SP;
    }
    return reg_push;
}

static uint8_t used_registers(void)
{
    uint32_t reg_push = 0;
    if (dis.use_r_a)
    {
        reg_push |= 1 << REG_A;
    }
    if (dis.use_r_bc)
    {
        reg_push |= 1 << REG_BC;
    }
    if (dis.use_r_de)
    {
        reg_push |= 1 << REG_DE;
    }
    if (dis.use_r_hl)
    {
        reg_push |= 1 << REG_HL;
    }
    if (dis.use_r_sp)
    {
        reg_push |= 1 << REG_SP;
    }
    if (dis.use_r_regfile)
    {
        reg_push |= 1 << REG_REGF;
    }
    return reg_push;
}

static uint8_t dirty_registers(void)
{
    uint32_t reg_push = 0;
    if (dis.dirty_r_a)
    {
        reg_push |= 1 << REG_A;
    }
    if (dis.dirty_r_bc)
    {
        reg_push |= 1 << REG_BC;
    }
    if (dis.dirty_r_de)
    {
        reg_push |= 1 << REG_DE;
    }
    if (dis.dirty_r_hl)
    {
        reg_push |= 1 << REG_HL;
    }
    if (dis.dirty_r_sp)
    {
        reg_push |= 1 << REG_SP;
    }
    reg_push |= 1 << REG_REGF;
    return reg_push;
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
    
    // bl fn
    int32_t rel_addr = ((intptr_t)arm_interworking_none(fn) - (intptr_t)outbuff) / 2 - 2;
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
    if ((int)urel_addr < -16777216 || (int)urel_addr > 16777214)
    {
        // shouldn't be possible -- playdate's memory is only 16 mb.
        assert(false);
    }
    #endif
    
    //JIT_DEBUG_MESSAGE("callsite address: %8x", outbuff);
    //JIT_DEBUG_MESSAGE("function address: %8x", fn);
    //JIT_DEBUG_MESSAGE("relative addr: %8x; sign %x, j2 %x, j1 %x\n", urel_addr, sign, j2, j1);
    
    WRITE_BUFF_32(
        0xF000D000 | (urel_addr & 0x7ff) | ((urel_addr << 5) & 0x03ff0000) | j2 | j1 | sign
    );
    
    return WRITE_BUFF_END();
}

// call function directly.
static uint8_t fasm_delegate(void* fn, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    
    // only need to save registers r0-r3
    // registers r4+ are callee-saved
    uint32_t reg_push = used_registers() & 0xF;
    
    // push {...}
    if (reg_push)
    {
        WRITE_BUFF_16(reg_push | 0xb400);
    }
    
    outbuff += fasm_bl(fn, FWD_BUFF());
    
    if (reg_push)
    {
        // pop {...}
        WRITE_BUFF_16(reg_push | 0xbc00);
    }
    
    return WRITE_BUFF_END();
}

static void frag_delegate(fn_type fn)
{
    dis.use_lr = 1;
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)fn,
        .produce = fasm_delegate
    );
}

static void frag_bl(fn_type fn)
{
    dis.use_lr = 1;
    dis.frag[dis.fragc++] = FRAG(
        .args = (void*)fn,
        .produce = fasm_bl
    );
}

static void frag_set_n(void)
{
    // TODO
}

static void frag_clear_n(void)
{
    // TODO
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

static void frag_armpush(uint16_t regs)
{
    if (bit(regs, 15) || bit(regs, 13))
    {
        assert(false);
    }
    
    if (regs == 0) return;
    
    if (!bits(regs, 8, 4))
    {
        frag_instr16(regs | 0xb400);
    }
    else
    {
        // TODO
        assert(false);
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
        frag_instr16(regs | 0xbd00);
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

// v: 0 if clear, 1 if set.
// b: the bit to set (0-7)
static void frag_set(unsigned b, sm83_oparg_t dst, int v)
{
    uint32_t instruction = (v)
        // orr.w
        ? 0xf0400000
        // bic
        : 0xf0200000;
    
    if (dst == OPARG_HLm)
    {
        use_register(dst, 1, 0);
        dis.use_r_flex = 1;
        
        assert(REG_HL >= 4); // need this to be able to copy hl to r0 for argument.
        frag_armpush((used_registers() | 2) & 0xe);
        
        // movs r0, r%hl
        frag_instr16(0x0000 | (REG_HL << 3));
        
        frag_bl((fn_type)opts.read);
        
        // bic/orr r1,r0,#(1<<b)
        frag_imm12c_rd8_rn16(instruction, 0, 1, false, 1, 32-b);
        
        // movs r0, r%hl
        frag_instr16(0x0000 | (REG_HL << 3));
        
        frag_bl((fn_type)opts.write);
        
        frag_armpop((used_registers() | 2) & 0xe);
        
        // we have to stop if we write to rom.
        if (!opts.fixed_bank) dis.done = 1;
    }
    else if (is_reg8(dst))
    {
        use_register(dst, 1, 1);
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

static void frag_inc(sm83_oparg_t arg)
{
    frag_clear_n();
    // TODO
}

static void frag_dec(sm83_oparg_t arg)
{
    frag_set_n();
    // TODO
}

// moves 16-bit immediate into the given register
// no assumptions are made about the contents of dst
static void frag_ld_imm16(int dstidx, uint16_t imm)
{
    #if 0
    // disabled because affecting the flags is scary.
    if (im <= 0xff)
    {
        // MOVS r%dst, imm
        frag_instr16(
            0x2000
            | (imm)
            | (dstidx << 8)
        );
    }
    else
    #endif
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

// moves 8-bit sm83 register into r0 or r%a
// dstidx must be the index of an arm register whose upper 24 bits can be assumed to be 0.
static void frag_store_rz_reg8(int dstidx, sm83_oparg_t src)
{
    assert(dstidx == REG_A || dstidx == REG_FLEX);
    assert(is_reg8(src));
    
    const uint8_t rn = reg_armidx(src);
    const uint8_t rd = dstidx;
    
    if (is_reghi(src))
    {
        // lsrs r%dst, r%src, 8
        frag_instr16(
            0x0A00
            | (rd << 0)
            | (rn << 3)
        );
    }
    else
    {
        // movs r%dst, r%src
        frag_instr16(
            0x0000
            | (rd << 0)
            | (rn << 3)
        );
        
        // uxtb r%dst, r%dst
        frag_instr16(
            0xB2C0
            | (rd << 3)
            | (rd << 0)
        );
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
        // and r%dst, r%dst, #0xff00
        frag_instr32(
            0xf400407f
            | (rd << 8)
            | (rn << 16)
        );
        
        // orr r%dst, r%src
        frag_instr16(
            0x4300
            | (rd << 0)
            | (rn << 3)
        );
    }
    else if (is_reghi(dst))
    {
        // uxtb r%dst,r%dst
        frag_instr16(
            0xb2c0
            | (rd << 0)
            | (rd << 3)
        );
        
         // orr r%dst, r%dst, r%src, lsl #8
        frag_instr32(
            0xEA402000
            | (rd << 16)
            | (rd << 8)
            | (rn << 0)
        );
    }
    else
    {
        assert(false);
    }
}

static void frag_ld(sm83_oparg_t dst, sm83_oparg_t src)
{
    if (dst == OPARG_HLmi)
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
    
    use_register(src, 1, 0);
    use_register(dst, 0, 1);
    
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
        dis.use_r_flex = 1;
        
        unsigned regs = used_registers() & 0xe;
        
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
            frag_instr16(0x0000 | (reg_armidx(src) << 3));
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
    else if (
        dst == OPARG_i16m || dst == OPARG_i8m || dst == OPARG_Cm
        || dst == OPARG_BCm || dst == OPARG_DEm || dst == OPARG_HLm
    )
    {
        assert(is_reg8(src));
        dis.use_r_flex = 1;
        
        // FIXME: frag_push/frag_pop should be its own instruction, because
        // if we clobber r1 and it's smear-written later... aaahh...
        // well, we always push r1 as a result, but that isn't *always* needed!
        
        frag_armpush((used_registers() | 2) & 0xe);
        
        switch (dst)
        {
        case OPARG_BCm:
        case OPARG_DEm:
        case OPARG_HLm:
            // movs r0, r%src
            frag_instr16(0x0000 | (reg_armidx(dst) << 3));
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
        if (src != OPARG_A)
        {
            // it's safe to clobber r1 with the write value
            // because we'll pop r1 -> a later if it is needed.
            frag_store_rz_reg8(1, src);
        }
        
        // call write
        frag_bl((fn_type)opts.write);
        
        // pop {...}
        frag_armpop((used_registers() | 2) & 0xe);
        
        // we have to stop now because of the possibility of a bankswap under our feet.
        if (!(dst == OPARG_i16m && sm83_addr_safe_write(immediate))
        || dst == OPARG_i8m || dst == OPARG_Cm
        || (opts.fixed_bank && (dst == OPARG_BCm || dst == OPARG_DEm || dst == OPARG_HLm)))
        {
            dis.done = true;
        }
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
                            0xf0400000, reg_armidx(dst), reg_armidx(dst), 0, immediate, 24
                        );
                    }
                }
                else
                {
                    // and r%dst, $ff00
                    frag_imm12c_rd8_rn16(
                        0xf0000000, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                    
                    // orr  r%dst, imm
                    if (immediate != 0)
                    {
                        frag_instr32(
                            0xF0400000
                            | (reg_armidx(dst) << 8)
                            | (reg_armidx(dst) << 16)
                            | immediate
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
                        // and r0, r%dst, #$ff00
                        frag_instr32(
                            0xf400407f
                            | (reg_armidx(dst) << 16)
                        );
                        
                        // orr r%dst, r0, r%dst, lsr #8
                        frag_instr32(
                            0xEA402010
                            | (reg_armidx(dst) << 8)
                        );
                    }
                    else
                    {
                        // uxtb r0, r%dst
                        frag_instr16(
                            0xB2C0
                            | (reg_armidx(dst) << 3)
                        );
                        
                        // orr r%dst, r0, r%dst, lsl #8
                        frag_instr32(
                            0xEA402000
                            | (reg_armidx(dst) << 8)
                        );
                    }
                }
                else if (is_reghi(dst) && is_reghi(src))
                {
                    // uxtb r%dst, r%dst
                    frag_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // push {r%src}
                    frag_armpush(1 << reg_armidx(src));
                    
                    // and r%src, $ff00
                    frag_imm12c_rd8_rn16(
                        0xf0000000, reg_armidx(src), reg_armidx(src), 0, 0xff, 24
                    );
                    
                    // orr r%dst, r%src
                    frag_instr16(
                        0x4300
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                    
                    // pop {r%src}
                    frag_armpop(1 << reg_armidx(src));
                }
                else if (is_reglo(dst) && is_reglo(src))
                {
                    // and r%dst, $ff00
                    frag_imm12c_rd8_rn16(
                        0xf0000000, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                    
                    frag_armpush(1 << reg_armidx(src));
                    
                    // uxtb r%src, r%src
                    frag_instr16(
                        0xB2C0
                        | (reg_armidx(src) << 3)
                        | (reg_armidx(src) << 0)
                    );
                    
                    // orr r%dst, r%src
                    frag_instr16(
                        0x4300
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                    
                    // pop {r%src}
                    frag_armpop(1 << reg_armidx(src));
                }
                else if (is_reghi(dst) && !is_reghi(src))
                {
                    
                    // uxtb r%dst, r%dst
                    frag_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // orr r%dst, r%dst, r%src, lsl #8
                    frag_instr32(
                        0xEA402000
                        | (reg_armidx(dst) << 16)
                        | (reg_armidx(dst) << 8)
                        | (reg_armidx(src) << 0)
                    );
                    
                    // OPTIMIZE -- can skip this if we know src's hi is 0.
                    // and r%dst, $ff00
                    frag_imm12c_rd8_rn16(
                        0xf0000000, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                }
                else if (!is_reghi(dst) && is_reghi(src))
                {
                    // and r%dst, $ff00
                    // and r%src, $ff00
                    frag_imm12c_rd8_rn16(
                        0xf0000000, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
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
            // movw r%dst, imm
            frag_instr32(
                0xF2400000
                | (reg_armidx(dst) << 8)
                | (bits(immediate, 0, 8) << 0)
                | (bits(immediate, 8, 3) << 12)
                | (bits(immediate, 11, 1) << 26)
                | (bits(immediate, 12, 4) << 16)
            );
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

static void frag_rotate(rotate_type rt, sm83_oparg_t dst)
{
    // TODO
}

static void frag_swap(sm83_oparg_t dst)
{
    assert(is_reg8(dst));
}

static void frag_jump(sm83_oparg_t condition, sm83_oparg_t addrmode)
{
    // TODO
}

static void frag_call(sm83_oparg_t condition)
{
    // TODO
}

static void frag_rst(unsigned x)
{
    // TODO
}

static void frag_ret(sm83_oparg_t condition)
{
    // TODO
}

static void frag_reti(void)
{
}

// convert to bcd
static void frag_daa(void)
{
}

// set carry flag
static void frag_scf(void)
{
}

// flip carry flag
static void frag_ccf(void)
{
}

// one's complement of A
static void frag_cpl(void)
{
}

static void frag_di(void)
{
}

static void frag_ei(void)
{
}

static void frag_add(sm83_oparg_t dst, sm83_oparg_t src, int carry)
{
}

static void frag_sub(sm83_oparg_t dst, sm83_oparg_t src, int carry)
{
}

static void frag_cp(sm83_oparg_t cmp)
{
}

static void frag_and(sm83_oparg_t src)
{
}

static void frag_or(sm83_oparg_t src)
{
}

static void frag_xor(sm83_oparg_t src)
{
}

static void frag_bit(unsigned b, sm83_oparg_t src)
{
}

static void frag_push(sm83_oparg_t src)
{
    use_register(src, 1, 0);
    use_register(OPARG_SP, 1, 1);
    
    if (src == OPARG_AF)
    {
        // TODO
        assert(false);
    }
    else if (is_reg16(src))
    {
        uint16_t regs = used_registers() & 0xe;
        
        frag_armpush(regs);
        
        // subs r%sp, 2 [T2]
        frag_instr16(0x3802 | (REG_SP << 8));
        
        // mov r%sp, r1
        frag_instr16(0x0000 | (reg_armidx(src) << 3) | 1);
        
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        
        // mov r0, r%sp
        frag_instr16(0x0000 | (REG_SP << 3));
        
        frag_bl((fn_type)opts.write);
        
        frag_armpop(regs);
    }
    else
    {
        assert(false);
    }
}

static void frag_pop(sm83_oparg_t dst)
{
    use_register(dst, 0, 1);
    use_register(OPARG_SP, 1, 1);
    
    if (dst == OPARG_AF)
    {
        // TODO
        assert(false);
    }
    else if (is_reg16(dst))
    {
        uint16_t regs = (used_registers() & ~(1 << reg_armidx(dst))) & 0xe;
        
        frag_armpush(regs);
        
        // mov r0, r%sp
        frag_instr16(0x0000 | (REG_SP << 3));
        
        // adds r%sp, 2 [T2]
        frag_instr16(0x3002 | (REG_SP << 8));
        
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        
        frag_bl((fn_type)opts.read);
        
        frag_armpop(regs);
        
        // mov r%dst, r0
        frag_instr16(0x0000 | reg_armidx(dst));
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
    return dis.done;
}

static void disassemble_begin(uint32_t gb_rom_offset, uint32_t gb_start_offset, uint32_t gb_end_offset)
{
    memset(&dis, 0, sizeof(dis));
    dis.fragcap = 0x100;
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
    dis.use_r_a = 1;
    dis.fragc = JIT_START_PADDING_ENTRY_C;
    dis.rom = (const uint8_t*)opts.rom + gb_rom_offset;
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
            return frag_rotate(ROT_LEFT_CARRY, OPARG_A);
            
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
            return frag_rotate(ROT_RIGHT_CARRY, OPARG_A);
        
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
            return frag_rotate(ROT_LEFT, OPARG_A);
            
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
            return frag_rotate(ROT_RIGHT, OPARG_A);
            
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
            return frag_ld(OPARG_HLm, OPARG_i8);
            
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
            return frag_jump(COND_nz, OPARG_i16);
            
        case 0xC3:
            return frag_jump(COND_none, OPARG_i16);
            
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
            return frag_jump(COND_z, OPARG_i16);
            
        case 0xCB:
            {
                uint8_t opx = sm83_next_byte();
                sm83_oparg_t arg = arg_from_op(opx);
                switch(opx/8)
                {
                case 0x00/8:
                    return frag_rotate(ROT_LEFT_CARRY, arg);
                case 0x08/8:
                    return frag_rotate(ROT_RIGHT_CARRY, arg);
                case 0x10/8:
                    return frag_rotate(ROT_LEFT, arg);
                case 0x18/8:
                    return frag_rotate(ROT_RIGHT, arg);
                case 0x20/8:
                    return frag_rotate(SHIFT_LEFT, arg);
                case 0x28/8:
                    return frag_rotate(SHIFT_RIGHT, arg);
                case 0x30/8:
                    return frag_swap(arg);
                case 0x38/8:
                    return frag_rotate(SHIFT_RIGHT_L, arg);
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
            return frag_jump(COND_nc, OPARG_i16);
            
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
            return frag_jump(COND_c, OPARG_i16);
            
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
            return frag_add(OPARG_SP, OPARG_i8, 0);
            
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
            return frag_di();
            
        case 0xF5:
            return frag_push(OPARG_AF);
            
        case 0xF6:
            return frag_or(OPARG_i8);
            
        case 0xF7:
            return frag_rst(6);
            
        case 0xF8:
            return frag_ld(OPARG_HL, OPARG_i8sp);
            
        case 0xF9:
            return frag_ld(OPARG_SP, OPARG_HL);
            
        case 0xFA:
            return frag_ld(OPARG_A, OPARG_i16m);
            
        case 0xFB:
            return frag_ei();
            
        case 0xFD:
            return frag_cp(OPARG_i8);
            
        case 0xFF:
            return frag_rst(7);
            
        default:
            dis.done = 1;
            frag_delegate(opts.illegal);
    }
}

// these next two functions are estimates / heuristics
// feel free to reimplement them.

struct regfile_io_strategy_t {
    uint16_t read_regs;
    uint16_t write_regs;
    
    // 0 if no registers accessed
    // 1 to access individually
    // 2 for load/store multiple
    int ldtype;
    int sttype;
};

#define SMEAR(a) (bitsmear_right(a) & ~1)

static int access_strategy_for_bits(uint16_t regs)
{
    uint32_t smearwaste = __builtin_popcount(SMEAR(regs) & ~(regs|1));
    switch (__builtin_popcount(regs))
    {
    case 0:
        return 0;
    case 1:
        return 1;
    case 2:
        return 1 + (smearwaste <= 1);
    case 3:
        return 1 + (smearwaste <= 3);
    default:
        return 2;
    }
    return 2;
}

static struct regfile_io_strategy_t get_regfile_io_strategy(void)
{
    struct regfile_io_strategy_t strat;
    uint32_t in = input_registers() & ~1;
    uint32_t out = dirty_registers() & ~1;
    
    assert(REG_FLEX == 0);
    uint32_t would_be_clobbered_if_writesmear_loadsingle = ~(in | dirty_registers()) & SMEAR(out);
    uint32_t writesmear_waste = SMEAR(out) & ~out;
    uint32_t readsmear_waste = SMEAR(in | (SMEAR(out) & ~dirty_registers())) & ~out & ~in;
    
    // should use cycle counting to calculate these weights
    int smear_advantage =
        __builtin_popcount(out)
        - __builtin_popcount(writesmear_waste)
        + __builtin_popcount(in)
        - __builtin_popcount(readsmear_waste) * 2;
    
    if (smear_advantage > 0)
    {
        // force load the non-dirty smeared out registers
        in |= SMEAR(out) & ~dirty_registers();
    }
    
    strat.ldtype = access_strategy_for_bits(in);
    strat.sttype = access_strategy_for_bits(out);
    
    // regardless of above reimplementation, the following must hold because
    // it satisfies important invariants regarding ldm/stm sequentiality.
    
    if (strat.sttype == 2)
    {
        if (~(in | dirty_registers()) & SMEAR(out))
        {
            strat.sttype = 1;
        }
    }
    
    strat.read_regs = in;
    strat.write_regs = out;
    if (strat.ldtype == 2) strat.read_regs = SMEAR(strat.read_regs);
    if (strat.sttype == 2) strat.write_regs = SMEAR(strat.write_regs);
    
    // TODO: mark any smeared registers as 'used'
    
    return strat;
}

static void disassemble_padding(void)
{
    // add some extra code at the start/end of a jit block
    // this will do things like e.g. ensure call convention correctness,
    // push/pop used registers, and load gb regs into/from arm regs.
    
    // epilogue_x: adds code to end of block.
    // prologue_x: adds code to start of block.
    
    // prologue size cannot exceed JIT_START_PADDING_ENTRY_C
    
    #define epilogue_16(x) frag_instr16(x)
    #define epilogue_32(x) frag_instr32(x)
    #define prologue_16(x) do { dis.frag[padc++] = FRAG(.args=(void*)(uintptr_t)(x), .produce=fasm_16); } while (0)
    #define prologue_32(x) do { dis.frag[padc++] = FRAG(.args=(void*)(uintptr_t)(x), .produce=fasm_32);  } while (0)
    
    size_t padc = 0;
    
    // we have to push r4-r7 if we use them -- they are callee-push registers.
    uint32_t reg_push = used_registers() & ~0xf;
    if (reg_push && ! dis.use_lr)
    {
        prologue_16(0xb400 | reg_push);
    }
    else if (dis.use_lr)
    {
        prologue_16(0xb500 | reg_push);
    }
    
    if (dis.use_r_regfile)
    {
        JIT_DEBUG_MESSAGE("r%d <- regfile: %8x", REG_REGF, (uintptr_t)(void*)opts.regs);
        
        // movw r1, >&regfile
        uintptr_t regf_addr = (uintptr_t)(void*)opts.regs;
        prologue_32(
            0xF2400000
            | (bits(regf_addr, 0, 8)  << 0)
            | (bits(regf_addr, 8, 3)  << 12)
            | (bits(regf_addr, 11, 1) << 26)
            | (bits(regf_addr, 12, 4) << 16)
            | (REG_REGF << 8)
        );
        
        // movt r1, <&regfile
        prologue_32(
            0xF2C00000
            | (bits(regf_addr, 16, 8)  << 0)
            | (bits(regf_addr, 24, 3)  << 12)
            | (bits(regf_addr, 27, 1) << 26)
            | (bits(regf_addr, 28, 4) << 16)
            | (REG_REGF << 8)
        );
    }
    
    struct regfile_io_strategy_t strat = get_regfile_io_strategy();
    
    switch (strat.ldtype)
    {
    case 0:
        break;
    case 1:
        if (strat.read_regs & (1 << REG_A))
        {
            prologue_16(
                0x6800
                | (offsetof(jit_regfile_t, a) << 6)
                | (REG_REGF << 3)
                | (REG_A << 0)
            );
        }
        
        if (strat.read_regs & (1 << REG_BC))
        {
            prologue_16(
                0x8800
                | (offsetof(jit_regfile_t, bc) << 5)
                | (REG_REGF << 3)
                | (REG_BC << 0)
            );
        }
        
        if (strat.read_regs & (1 << REG_DE))
        {
            prologue_16(
                0x8800
                | (offsetof(jit_regfile_t, de) << 5)
                | (REG_REGF << 3)
                | (REG_DE << 0)
            );
        }
        
        if (strat.read_regs & (1 << REG_HL))
        {
            prologue_16(
                0x8800
                | (offsetof(jit_regfile_t, hl) << 5)
                | (REG_REGF << 3)
                | (REG_HL << 0)
            );
        }
        
        if (strat.read_regs & (1 << REG_SP))
        {
            prologue_16(
                0x8800
                | (offsetof(jit_regfile_t, sp) << 5)
                | (REG_REGF << 3)
                | (REG_SP << 0)
            );
        }
        break;
    case 2:
        // [A6.7.40] LDM r%regf, { ... }
        assert(strat.read_regs == SMEAR(strat.read_regs));
        prologue_16(
            0xC800 | strat.read_regs | (REG_REGF << 8)
        );
    }
    
    switch (strat.sttype)
    {
    case 0: // no write/read necessary.
        break;
    case 1: // write individually
        if (strat.write_regs & (1 << REG_A))
        {
            // write value of A back to regfile
            // strb r%a, [r%regf, offsetof(jit_regfile, a)]
            epilogue_16(
                0x7000
                | ((offsetof(jit_regfile_t, a) / sizeof(uint32_t)) << 6)
                | (REG_REGF << 3)
                | (REG_A << 0)
            );
        }
        
        if (strat.write_regs & (1 << REG_BC))
        {
            // strh r%bc, [r%regf, offsetof(jit_regfile, bc)]
            epilogue_16(
                0x8000
                | ((offsetof(jit_regfile_t, bc)/2) << 6)
                | (REG_REGF << 3)
                | (REG_BC << 0)
            );
        }
        
        if (strat.write_regs & (1 << REG_DE))
        {
            epilogue_16(
                0x8000
                | ((offsetof(jit_regfile_t, de)/2) << 6)
                | (REG_REGF << 3)
                | (REG_DE << 0)
            );
        }
        
        if (strat.write_regs & (1 << REG_HL))
        {
            epilogue_16(
                0x8000
                | ((offsetof(jit_regfile_t, hl)/2) << 6)
                | (REG_REGF << 3)
                | (REG_HL << 0)
            );
        }
        
        if (strat.write_regs & (1 << REG_SP))
        {
            epilogue_16(
                0x8000
                | ((offsetof(jit_regfile_t, sp)/2) << 6)
                | (REG_REGF << 3)
                | (REG_SP << 0)
            );
        }
        break;
    case 2: // we can use the store-multiple instruction
    
        assert(strat.write_regs == SMEAR(strat.write_regs));
        
        // mustn't clobber anything...
        assert (!(~(strat.read_regs | dirty_registers()) & strat.write_regs));
    
        // STM r%regf, { ... }
        epilogue_16(
            0xC000 | strat.write_regs | (REG_REGF << 8)
        );
        break;
    }
    
    // pop {...}
    frag_armpop(reg_push | (dis.use_lr << 14));
    
    // ret
    epilogue_16(0x4770);
    
    assert(padc <= JIT_START_PADDING_ENTRY_C);
}

static void* disassemble_end(void)
{
    if (dis.error)
    {
        if (dis.frag) free(dis.frag);
        return NULL;
    }
    
    disassemble_padding();
    
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
    
    JIT_DEBUG_MESSAGE("base address: %8x", out);
    uint16_t* outb = out;
    
    for (size_t i = 0; i < dis.fragc; ++i)
    {
        outb += dis.frag[i].produce(dis.frag[i].args, outb);
    }
    
    #ifdef JIT_DEBUG
    #define printf opts.playdate->system->logToConsole
    //JIT_DEBUG_MESSAGE("arm code: ");
    const char* outmsg;
    size_t fragi = 0;
    size_t armseek = 0;
    printf("Used: %02x; Dirty: %02x", used_registers(), dirty_registers());
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
                printf("\n; %s", outmsg);
            }
        }
        const uint16_t* armprev = arm;
        arm = armd(arm, &outmsg);
        if (arm && arm == armprev+1)
        {
            printf("%04x ; %s", *armprev, outmsg);
        }
        else if (arm && arm == armprev+2)
        {
            printf("%04x%04x ; %s", armprev[0], armprev[1], outmsg);
        }
        else
        {
            opts.playdate->system->logToConsole("DISASM ERROR");
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

void jit_invalidate_cache()
{
    #if TARGET_PLADATE
    SCB_InvalidateDCache();
    SCB_InvalidateICache();
    SCB_InvalidateDCache();
    SCB_InvalidateICache();
    #endif
}

jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank)
{
    jit_fn f = get_jit_fn(gb_addr, gb_bank);
    if (f) return f;
    
    // no existing fn -- create one.
    jit_fn fn = jit_compile(gb_addr, gb_bank);
    if (!fn) return NULL;
    jit_invalidate_cache();
    return fn;
}

