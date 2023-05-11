#include "jit.h"
#include "armd.h"
#include "sm38d.h"

/*
    Useful resources when developing this JIT:
    - Arm v7-m architecture reference manual: 
        - https://developer.arm.com/documentation/ddi0489/b/
        - This will help convert between arm opcodes and hexadecimal.
    - Godbolt copmiler: https://godbolt.org/z/ornhbWjee
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
            - r5 = SP (high 2 bytes are always 0)
            - r6 = Carry (only 1 bit is used)
            - r7 = z, n, and h flags (see comment in jit.h)
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
*/

/*
    OPTIMIZATION IDEAS
    - these sm83 commands don't exist: ld bc,hl; ld de,bc; and so on.
        - they are emulated in sm83 by instruction pairs e.g. push hl, pop bc; ld b,h; ld c, l
        - we can identify this and reduce to just one arm mov instruction.
        
    - we can analyze the rst routines and inline them
    
    - store status flags in r2, r3;
      that way we can clobber them freely during function calls
      most of the time.
*/

#ifndef JIT_DEBUG
    #define JIT_DEBUG
#endif

#define SETFLAGS 1

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
#define THUMB16_ADD_3   0x1C00
#define THUMB16_SUB_3   0x1E00
#define THUMB16_LSL_IMM 0x0000
#define THUMB16_LSR_IMM 0x0800
#define THUMB16_ASR_IMM 0x1000
#define THUMB16_ADD_8   0x3000
#define THUMB16_SUB_8   0x3800
#define THUMB16_ORR_REG 0x4300
#define THUMB16_ADD_REG_T2 0x4400 /* rdn0127_rm_3456 */
#define THUMB16_UXTH    0xB280
#define THUMB16_UXTB    0xB2C0

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
#define THUMB32_BFI     0xf3000000

#define THUMB32_ORR_REG 0xea400000

// all sm83 registers
#define REGS_SM83 (REG_A | REG_BC | REG_DE | REG_HL | REG_SP | REG_Carry | REG_nZ | REG_NH)
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
    uint32_t reg_push = REGS_NONFLEX & 0xF;
    
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
    
    // we have to stop if we write to rom.
    if (!opts.fixed_bank) dis.done = 1;
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

static void frag_add_rd_rn(
    uint8_t rdn, uint8_t rm
);
{
    frag_instr16(
        THUMB16_ADD_REG_T2
        | (rm << 4)
        | (bits(rdn, 0, 3))
        | (bit(rdn, 3) << 7)
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

static void frag_ubfx(
    uint8_t rd,
    uint8_t rn,
    uint8_t pos,
    uint8_t width
)
{
    frag_instr16(
        0xf3c00000
        | (rd << 8)
        | (rn << 16)
        | (width & 0b11111)
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
    frag_instr16(
        0xf3400000
        | (rd << 8)
        | (rn << 16)
        | (width & 0b11111)
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
    
    return instruction
        | (rm)
        | (rd << 8)
        | (rn << 16)
        | (shift_type << 4)
        | (bits(shift_x, 0, 2) << 6)
        | (bits(shift_x, 2, 3) << 12);
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
    #if 0
    if (n && h)
    {
        // uxth r%znh, r%znh
        frag_instr16(0xb280 | (REG_NH << 3) | REG_NH);
        
        // orr.w r%znh, r%znh, #$1020
        frag_imm12c_rd8_rn16(
            THUMB32_ORR_IMM,
            REG_NH, REG_NH, false, 0x81, 27
        );
    }
    else if (n && !h)
    {
        // uxtb r%znh, r%znh
        frag_instr16(
            0xB2C0
            | (REG_NH << 3)
            | (REG_NH << 0)
        );
        
        // orr.w r%znh, r%znh, #$20
        frag_imm12c_rd8_rn16(
            THUMB32_ORR_IMM,
            REG_NH, REG_NH, false, 0x20, 0
        );
    }
    else if (!n && h)
    {
        // bic.w r%znh, r%znh, #$20
        frag_imm12c_rd8_rn16(
            THUMB32_BIC_IMM,
            REG_NH, REG_NH, false, 0x20, 24
        );
        
        // we could avoid this if we are more careful with how we read h implicitly.
        // uxth r%znh, r%znh
        frag_instr16(0xb280 | (REG_NH << 3) | REG_NH);
        
        // orr.w r%znh, r%znh, #$1000
        frag_imm12c_rd8_rn16(
            THUMB32_ORR_IMM,
            REG_NH, REG_NH, false, 0x10, 24
        );
    }
    else if (!n && !h)
    {
        // and.w r%znh, r%znh, #1
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM,
            REG_NH, REG_NH, false, 1, 0
        );
    }
    #endif
}

// v: 0 if clear, 1 if set.
// b: the bit to set (0-7)
static void frag_set(unsigned b, sm83_oparg_t dst, int v)
{
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
    const uint16_t op = (dec)
        ? 0x3801
        : 0x3001;
        
    const uint32_t op32 = (dec)
        ? 0xf1a00000
        : 0xf5000000;
    
    if (is_reg16(arg))
    {
        // adds r%arg, 1 [T2]
        frag_instr16(op | (reg_armidx(arg) << 8));
        
        // uxth r%arg, r%arg
        frag_instr16(0xb280 | (reg_armidx(arg) << 3) | reg_armidx(arg));
        
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
                    THUMB16_LSL_IMM, REG_NH, reg, 16
                );
            }
            #endif
            
            // adds/subs r%arg, 1 [T2]
            frag_instr16(op | (reg_armidx(arg) << 8));
            
            #if SETFLAGS
            if (dec)
            {
                frag_instr16_rd0_rm3_imm5(
                    THUMB16_LSL_IMM, REG_NH, reg, 24
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
            frag_instr16(op | (reg_armidx(arg) << 8));
            
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
            frag_imm12c_rd8_rn16(
                op32, reg_armidx(arg), reg_armidx(arg), false, 0x1, 8
            );
            
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
        frag_instr16(op);
        
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

// moves 8-bit sm83 register into r0 or r%a
// dstidx must be the index of an arm register whose upper 24 bits can be assumed to be 0.
static void frag_store_rz_reg8(int dstidx, sm83_oparg_t src)
{
    assert(dstidx == REG_A || dstidx == REG_FLEX);
    assert(is_reg8(src));
    
    const uint8_t rn = reg_armidx(src);
    const uint8_t rd = dstidx;
    
    if (src == REG_A)
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
        // movs r%dst, r%src
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, rd, rn);
        
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
    else if (
        dst == OPARG_i16m || dst == OPARG_i8m || dst == OPARG_Cm
        || dst == OPARG_BCm || dst == OPARG_DEm || dst == OPARG_HLm
    )
    {
        assert(is_reg8(src));
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
        if (src != OPARG_A)
        {
            // it's safe to clobber r1 with the write value
            // because we'll pop r1 -> a later if it is needed.
            frag_store_rz_reg8(1, src);
        }
        
        // call write
        frag_bl((fn_type)opts.write);
        
        // pop {...}
        frag_armpop(REGS_NONFLEX & 0xF);
        
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
                        THUMB32_AND_IMM, reg_armidx(src), reg_armidx(src), 0, 0xff, 24
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
                        THUMB32_AND_IMM, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
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
                        THUMB32_AND_IMM, reg_armidx(dst), reg_armidx(dst), 0, 0xff, 24
                    );
                }
                else if (!is_reghi(dst) && is_reghi(src))
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

static void frag_rotate_helper(rotate_type rt, uint8_t regdst, uint8_t reg, uint8_t carry_in, uint8_t carry_out, bool setz)
{
    const uint8_t scratchreg = (reg == 0) ? 1 : 0;
    switch (rt)
    {
    case ROT_LEFT:
    case SHIFT_LEFT:
    case ROT_LEFT_CARRY:
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, reg, reg, 1
        );
        
        if (rt == ROT_LEFT_CARRY)
        {
            frag_instr16_rd0_rm3(
                THUMB16_ORR_REG, reg, carry_in
            );
        }
        else if (rt == ROT_LEFT)
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, reg, reg, reg, IMM_SHIFT_LSR, 8
            );
        }
        
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, regdst, reg
        );
        break;
    
    case SHIFT_RIGHT:
    case ROT_RIGHT:
    case ROT_RIGHT_CARRY:
        if (rt == ROT_RIGHT)
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, reg, reg, reg, IMM_SHIFT_LSL, 8
            );
        }
        else if (rt == ROT_RIGHT_CARRY)
        {
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, reg, reg, carry, IMM_SHIFT_LSL, 8
            );
        }
        
        frag_ubfx(regdst, reg, 1, 8);
        break;
        
    case SHIFT_RIGHT_L:
        frag_sbfx(reg, reg, 1, 8);
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
}

static void frag_rotate(rotate_type rt, sm83_oparg_t dst,  bool setz)
{
    assert(is_reg8(dst) || dst == OPARG_HLm);
    
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
    // rely on REG_NH containing the carry bit,
    // which some rotations need to read.
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
    case ROT_LEFT_CARRY:
        frag_ubfx(REG_Carry, reg, 7+bit_offset, 1);
        break;
        
    case SHIFT_RIGHT:
    case SHIFT_RIGHT_L:
    case ROT_LEFT:
    case ROT_LEFT_CARRY:
        frag_ubfx(REG_Carry, reg, 0+bit_offset, 1);
        break;
    }
    #endif
    
    if (dst == OPARG_A)
    {
        frag_rotate_helper(rt, OPARG_A, OPARG_A, carry_in, carry_out, setz);
    }
    else if (is_reglo(dst))
    {
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, 0, reg
        );
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM, reg, reg, false, 0xff, 24
        );
        frag_rotate_helper(rt, 0, 0, carry_in, carry_out, setz);
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, reg, 0
        );
    }
    else if (is_reghi(dst))
    {
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, 0, reg, 8
        );
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, reg, reg
        );
        frag_rotate_helper(rt, 0, 0, carry_in, carry_out, setz);
        frag_rd8_rn16_rm0_shift(
            THUMB32_ORR_REG, reg, reg, 0, IMM_SHIFT_LSL, 8
        );
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
    assert(is_reg8(dst) || dst == OPARG_HLm);
    
    if (dst == OPARG_A)
    {
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, 0, REG_A, 4
        );
        
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, REG_A, REG_A, 4
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, REG_A, REG_A
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, REG_A, 0
        );
        
        #if SETFLAGS
        // f: z=*
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, REG_nZ, REG_A
        );
        #endif
    }
    else if (is_reglo(dst))
    {
        // TODO: optimize
        frag_ubfx(0, reg_armidx(dst), 4, 4);
        
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, REG_nZ, reg_armidx(dst), 4
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, REG_nZ, 0
        );
        
        // zero-out low byte
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM, reg_armidx(dst), reg_armidx(dst), false, 0xff, 24
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, REG_nZ, REG_nZ
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, reg_armidx(dst), REG_nZ
        );
        
        // z=* (already done, for free)
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
    }
    else if (dst == OPARG_HLm)
    {
        frag_pre_readwrite_hlm();
        
        // r0 now contains (HL)
        
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSR_IMM, 1, 0, 4
        );
        
        frag_instr16_rd0_rm3_imm5(
            THUMB16_LSL_IMM, 0, 0, 4
        );
        
        // OPT: this can be skipped if .write() takes uint32_t
        frag_instr16_rd0_rm3(
            THUMB16_UXTB, 0, 0
        );
        
        frag_instr16_rd0_rm3(
            THUMB16_ORR_REG, 1, 0
        );
        
        #if SETFLAGS
        // f: z=*
        frag_instr16_rd0_rm3(
            THUMB16_MOV_REG, REG_nZ, 1
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
    const uint8_t regs = (REGS_NONFLEX & ~(1 << REG_A)) & 0xe;
    frag_armpush(regs);
    
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
        REG_Carry, REG_Carry, 0, 1, 0
    );
    frag_set_nh(0, 0);
    #endif
}

static void frag_edi(bool enable)
{
    frag_ld_imm16(0, enable);
    
    // str r0, [r%regf,#...]
    frag_instr16_rd0_rm3_imm5(
        0x6000,
        0,
        REG_REGF,
        offsetof(jit_regfile_t, ime) >> 2
    );
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
    uint8_t op16 = THUMB16_ORR_REG;
    uint8_t op32 = THUMB32_ORR_REG;
    uint8_t op32imm = THUMB32_ORR_IMM;
    
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
    
    const uint8_t reg = reg_armidx(src);
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
        frag_ld_imm16(REG_nZ, REG_A)
        #endif
    }
    else if (is_reglo(src))
    {
        frag_instr16_rd0_rm3(
            op16, REG_A, reg
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
            op32, REG_A, REG_A, reg, IMM_SHIFT_LSR, 8
        );
        
        #if SETFLAGS
        frag_ld_imm16(REG_nZ, REG_A)
        #endif
    }
    else
    {
        assert(false);
    }
    
    #if SETFLAGS
    switch (arith)
    {
    case AND:
        frag_set_nh(0, 1);
        break;
        
    case OR:
    case XOR:
        frag_set_nh(0, 0);
        break;
    
    default:
        break;
    }
    #endif
}

static void frag_arithmetic(sm83_oparg_t dst, sm83_oparg_t src, arithop_t arithop)
{
    const bool carry_in = (arithop == ADC || arithop == SBC);
    const bool subtract = (arithop == SUB || arithop == SBC || arithop == CP);
    if (carry_in)
    {
        // optimization: we could set the (arm) carry bit flag here to the value in REG_C
        // then we can use the adc/sbc operations.
    }
    
    #if SETFLAGS
    if (dst != REG_HL || src != REG_HL)
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
        if (src == OPARG_i8m)
        {
            immediate = sm83_next_byte();
            // note: we can't replace with inc/dec if immediate is 1, because flags are different.
            
            const uint16_t op16_3 = (subtract)
                ? THUMB16_SUB_3
                : THUMB16_ADD_3;
            
            const uint16_t op16_8 = (subtract)
                ? THUMB16_SUB_8
                : THUMB16_ADD_8;
                
            const uint16_t op32_8 = (subtract)
                ? THUMB32_ADD_IMM_T3
                : THUMB32_SUB_IMM_T3;
                
            #if SETFLAGS
            // nh[3] <- REG_A
            frag_rd8_rn16_rm0_shift(
                THUMB32_ORR_REG, REG_NH, REG_NH, REG_A, IMM_SHIFT_LSL, 24
            );
            #endif
            
            if (immediate == immediate & 0b111)
            {
                // use op 3
                frag_instr16_rd0_rm3(
                    op16_3 | (imm << 6),
                    dstreg, REG_A
                );
            }
            else if (dstreg == REG_A)
            {
                frag_instr16(
                    op16_8 | (REG_A << 8) | immediate
                );
            }
            else
            {
                frag_imm12c_rd8_rn16(
                    op32_8, dstreg, REG_A, false, immediate, 0
                );
            }
            
            if (carry_in)
            {
                #if SETFLAGS
                // nh[1] <- carry in
                frag_rd8_rn16_rm0_shift(
                    THUMB32_ORR_REG, REG_NH, REG_NH, REG_Carry, IMM_SHIFT_LSL, 8
                );
                #endif
                
                // r%a += c
                frag_instr16_rd0_rn3_rm6(THUMB16_ADD_REG_T1, REG_A, REG_A, REG_Carry);
            }
            
            #if SETFLAGS
            // c <- carry out
            frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_Carry, dstreg, 8);
            
            // nh[2] <- imm
            frag_imm12c_rd8_rn16(
                THUMB32_ORR_IMM,
                REG_NH, REG_NH, false, imm, 16
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
                    frag_set_nh(1, 0);
                    #endif
                    break;
                    
                case SBC:
                    frag_sbfx(REG_A, Reg_Carry, 0, 1);
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
    frag_arithmetic(dst, src, CP);
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
        frag_instr16(0x3802 | (REG_SP << 8));
        
        frag_armpush(regs);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_Carry);
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 1, REG_nZ);
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 2, REG_NH);
        frag_bl((fn_type)jit_regfile_get_f);
        frag_rd8_rn16_rm0_shift(
            THUMB32_ORR_REG, 1, 0, REG_A, IMM_SHIFT_LSL, 8
        );
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_SP);
        frag_bl((fn_type));
        frag_bl((fn_type)opts.write16);
        frag_armpop(regs);
    }
    else if (is_reg16(src))
    {
        uint16_t regs = REGS_NONFLEX & 0xF;
        
        frag_armpush(regs);
        
        // subs r%sp, 2 [T2]
        frag_instr16(0x3802 | (REG_SP << 8));
        
        // mov r1, r%src
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 1, reg_armidx(src));
        
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        
        // mov r0, r%sp
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_SP);
        
        frag_bl((fn_type)opts.write16);
        
        frag_armpop(regs);
    }
    else
    {
        assert(false);
    }
}

static void frag_pop(sm83_oparg_t dst)
{
    const uint16_t regs = (REGS_NONFLEX & ~(1 << reg_armidx(dst))) & 0xf;
    
    if (dst == OPARG_AF)
    {
        frag_armpush(regs);
        
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_SP);
        
        frag_bl((fn_type)opts.read16);
        
        // adds r%sp, 2 [T2]
        frag_instr16(0x3002 | (REG_SP << 8));
        
        // nh <- r0 << 6
        frag_instr16_rd0_rm3_imm5(THUMB16_LSL_IMM, REG_NH, 0, 6);
        
        // a <- r0 >> 8
        frag_instr16_rd0_rm3_imm5(THUMB16_LSR_IMM, REG_A, 0, 8);
        
        // nh |= (r0 >> 1)
        frag_rd8_rn16_rm0_shift(
            THUMB32_ORR_REG, REG_NH, REG_NH, 0, IMM_SHIFT_LSR, 1
        );
        
        // c <- bit(r0, 4)
        frag_ubfx(REG_Carry, 0, 4, 1);
        
        // r%nz <- bit(r0, 7)
        frag_imm12c_rd8_rn16(
            THUMB32_AND_IMM,
            REG_nZ, 0, false, 0x80, 0
        );
        
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        
        // !z <- r%nz
        frag_imm12c_rd8_rn16(
            THUMB32_EOR_IMM,
            REG_nZ, REG_nZ, false, 0x80, 0
        );
        
        frag_armpop(regs);
    }
    else if (is_reg16(dst))
    {
        frag_armpush(regs);
        
        // mov r0, r%sp
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, 0, REG_SP);
        
        frag_bl((fn_type)opts.read);
        
        // adds r%sp, 2 [T2]
        frag_instr16(0x3002 | (REG_SP << 8));
        
        frag_armpop(regs);
        
        // uxth r%sp, r%sp
        frag_instr16(0xb280 | (REG_SP << 3) | REG_SP);
        
        // mov r%dst, r0
        frag_instr16_rd0_rm3(THUMB16_MOV_REG, reg_armidx(dst), 0);
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
            return frag_rotate(ROT_LEFT_CARRY, OPARG_A, false);
            
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
            return frag_rotate(ROT_RIGHT_CARRY, OPARG_A, false);
        
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
                    return frag_rotate(ROT_LEFT_CARRY, arg, true);
                case 0x08/8:
                    return frag_rotate(ROT_RIGHT_CARRY, arg, true);
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
            return frag_edi(0);
            
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
            return frag_edi(1);
            
        case 0xFD:
            return frag_cp(OPARG_i8);
            
        case 0xFF:
            return frag_rst(7);
            
        default:
            dis.done = 1;
            frag_delegate(opts.illegal);
    }
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
    uint32_t reg_push = REGS_ALL & ~0xf;
    if (reg_push && ! dis.use_lr)
    {
        prologue_16(0xb400 | reg_push);
    }
    else if (dis.use_lr)
    {
        prologue_16(0xb500 | reg_push);
    }
    
    // [A6.7.40] LDM r0, { ... }
    prologue_16(
        0xC800 | REGS_SM83 | (0 << 8)
    );
    
    // push { r0 }
    prologue_16(0xb400 | (1 << 0));
    
    // pop {r0}
    epilogue_16(0xbc00 | (1 << 0));
    
    // STM r%regf, { ... }
    epilogue_16(
        0xC000 | REGS_SM83 | (0 << 8)
    );
    
    
    // pop {...}
    frag_armpop(reg_push | (dis.use_lr << 14));
    
    // bx lr
    epilogue_16(0x4770);
    
    // TODO: figure out when this is needed exactly.
    // nop
    epilogue_16(0xbf00);
    
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

