#include "jit.h"
#include "armd.h"

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
#define REG_REGF 0
#define REG_FLEX REG_REGF

// these can be edited without loss of correctness.
// keep in mind that 5+ are caller-saved
// these are the arm registers used to store the given z80 registers
#define REG_A 1
#define REG_BC 3
#define REG_DE 4
#define REG_HL 2
#define REG_SP 5

#define offsetof_hi(a, b) (offsetof(a, b) + 1)

typedef void (*fn_type)(void);

typedef struct
{
    uint16_t addr;
    uint16_t bank;
    jit_fn fn;
} jit_ht_entry;

// suffix: m = memory (dereference)
//         mi = dereference increment
//         md = dereference decrement
//         sp = plus stack pointer
typedef enum
{
    OPARG_A, OPARG_B, OPARG_C, OPARG_D, OPARG_E, OPARG_H, OPARG_L,
    OPARG_AF, OPARG_BC, OPARG_DE, OPARG_HL, OPARG_SP,
    OPARG_Cm, OPARG_BCm, OPARG_DEm, OPARG_HLm, OPARG_HLmi, OPARG_HLmd,
    OPARG_i8, OPARG_i16, OPARG_i8sp,
    OPARG_i8m, OPARG_i16m,
    COND_none, COND_z, COND_c, COND_nz, COND_nc,
    ADDR_r8, ADDR_d16, ADDR_HL // relative 8; direct 16; indirect HL
} oparg;

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
    JIT_DEBUG_MESSAGE("memset.\n");
    memset(jit_hashtable, 0, sizeof(jit_hashtable));
    JIT_DEBUG_MESSAGE("memfix.\n");
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

typedef struct {
    void* args;
    
    // if `outbuff` is NULL, don't actually write anything.
    // return size of (what would be) written
    uint8_t (*produce)(void* args, uint16_t* outbuff);
    
    #ifdef JIT_DEBUG
    const uint8_t* romsrc;
    uint8_t length; // this is set from produce() and recorded for debugging purposes
    #endif
} armop;

// disassembly state.
static struct {
    armop* arm;
    uint16_t armc;
    uint16_t armcap;
    const uint8_t* romstart;
    const uint8_t* rom;
    const uint8_t* romend;
    unsigned done : 1;
    unsigned error : 1;
    
    // registers assigned
    unsigned use_r_a: 1;
    unsigned init_r_a : 1;
    unsigned dirty_r_a : 1;
    unsigned use_r_bc: 1;
    unsigned init_r_bc : 1;
    unsigned dirty_r_bc : 1;
    unsigned use_r_de: 1;
    unsigned init_r_de : 1;
    unsigned dirty_r_de : 1;
    unsigned use_r_hl: 1;
    unsigned init_r_hl : 1;
    unsigned dirty_r_hl : 1;
    unsigned use_r_sp: 1;
    unsigned init_r_sp : 1;
    unsigned dirty_r_sp : 1;
    unsigned use_r_regfile : 1;
    unsigned use_r_flex : 1;
    unsigned use_lr : 1;
} dis;

static uint8_t dis_read_rom_byte(void)
{
    return *(dis.rom++);
}

static uint16_t dis_read_rom_hword(void)
{
    uint16_t x = dis_read_rom_byte();
    x |= dis_read_rom_byte() << 8;
    return x;
}

static int is_reghi(oparg reg)
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


static int is_reglo(oparg reg)
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

static int is_reg8(oparg reg)
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

static int is_reg16(oparg reg)
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

static int reg_armidx(oparg reg)
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
    default:
        assert(false);
        return 0;
    }
}

// reg must be one of OPARG_A, OPARG_BC, OPARG_DE, OPARG_HL.
static void use_register(oparg reg, int dependency, int dirty)
{
    switch (reg)
    {
        case OPARG_AF:
            // fallthrough
        case OPARG_A:
            if (dependency && !dis.use_r_a)
            {
                dis.init_r_a = 1;
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
                dis.init_r_bc = 1;
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
                dis.init_r_de = 1;
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
                dis.init_r_hl = 1;
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
                dis.init_r_sp = 1;
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

static uint8_t disp_skip(void* args, uint16_t* outbuff)
{
    return 0;
}

static uint8_t disp_16(void* args, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    WRITE_BUFF_16((uintptr_t)args);
    return WRITE_BUFF_END();
}

static uint8_t disp_32(void* args, uint16_t* outbuff)
{
    WRITE_BUFF_INIT();
    WRITE_BUFF_32((uintptr_t)args);
    return WRITE_BUFF_END();
}

#ifdef JIT_DEBUG
    #define ARMOP(...) ({armop op = {__VA_ARGS__}; op.romsrc = NULL; op.length = 0; op;})
#else
        #define ARMOP(...) ({armop op = {__VA_ARGS__}; op;})
#endif

static void dis_instr16(uint16_t instr)
{
    dis.arm[dis.armc++] = ARMOP(
        .args = (void*)(uintptr_t)instr,
        .produce = disp_16
    );
}

static void dis_instr32(uint32_t instr)
{
    dis.arm[dis.armc++] = ARMOP(
        .args = (void*)(uintptr_t)instr,
        .produce = disp_32
    );
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

static uint8_t disp_bl(void* fn, uint16_t* outbuff)
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
    
    JIT_DEBUG_MESSAGE("callsite address: %8x", outbuff);
    JIT_DEBUG_MESSAGE("function address: %8x", fn);
    JIT_DEBUG_MESSAGE("relative addr: %8x; sign %x, j2 %x, j1 %x\n", urel_addr, sign, j2, j1);
    
    WRITE_BUFF_32(
        0xF000D000 | (urel_addr & 0x7ff) | ((urel_addr << 5) & 0x03ff0000) | j2 | j1 | sign
    );
    
    return WRITE_BUFF_END();
}

// call function directly.
static uint8_t disp_delegate(void* fn, uint16_t* outbuff)
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
    
    outbuff += disp_bl(fn, FWD_BUFF());
    
    if (reg_push)
    {
        // pop {...}
        WRITE_BUFF_16(reg_push | 0xbc00);
    }
    
    return WRITE_BUFF_END();
}

static void dis_delegate(fn_type fn)
{
    dis.use_lr = 1;
    dis.arm[dis.armc++] = ARMOP(
        .args = (void*)fn,
        .produce = disp_delegate
    );
}

static void dis_bl(fn_type fn)
{
    dis.arm[dis.armc++] = ARMOP(
        .args = (void*)fn,
        .produce = disp_bl
    );
}

static void dis_set_n(void)
{
    // TODO
}

static void dis_clear_n(void)
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

static void dis_inc(oparg arg)
{
    dis_clear_n();
    // TODO
}

static void dis_dec(oparg arg)
{
    dis_set_n();
    // TODO
}

static void dis_ld(oparg dst, oparg src)
{
    if (dst == OPARG_HLmi)
    {
        dis_ld(OPARG_HLm, src);
        dis_inc(OPARG_HL);
        return;
    }
    else if (dst == OPARG_HLmd)
    {
        dis_ld(OPARG_HLm, src);
        dis_dec(OPARG_HL);
        return;
    }
    else if (src == OPARG_HLmi)
    {
        dis_ld(dst, OPARG_HLm);
        dis_inc(OPARG_HL);
        return;
    }
    else if (src == OPARG_HLmd)
    {
        dis_ld(dst, OPARG_HLm);
        dis_dec(OPARG_HL);
        return;
    }
    
    use_register(src, 1, 0);
    use_register(dst, 0, 1);
    
    unsigned immediate = 0;
    if (src == OPARG_i8 || src == OPARG_i8m || src == OPARG_i8sp) immediate = dis_read_rom_byte();
    if (src == OPARG_i16 || src == OPARG_i16m) immediate = dis_read_rom_hword();

    if (is_reg8(dst))
    {
        if (src == OPARG_i8)
        // e.g. ld c, $50
        {
            if (dst == OPARG_A)
            {
                // movs r%a, imm
                // cf. ARMv7-M Architecture Reference Manual A6-148, Encoding T1
                dis_instr16(0x2000 | (immediate) | (reg_armidx(dst) << 8));
            }
            else
            {
                if (is_reghi(dst))
                {
                    // [A6.7.149] uxtb r%dst, r%dst
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // [A6.7.90] orr  r%dst, r%dst, imm<<8
                    if (immediate != 0)
                    {
                        unsigned imm_encoding = thumbexpand_encoding(immediate, 24);
                        JIT_DEBUG_MESSAGE("thumbexpenc: %8x", imm_encoding);
                        dis_instr32(
                            0xf0400000
                            | (bits(imm_encoding, 11, 1) << 26)
                            | (reg_armidx(dst) << 16)
                            | (bits(imm_encoding, 8, 3) << 12)
                            | (reg_armidx(dst) << 8)
                            | (bits(imm_encoding, 0, 8) << 0)
                        );
                    }
                }
                else
                {
                    // and r%dst, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(dst) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(dst) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                    
                    // orr  r%dst, imm
                    if (immediate != 0)
                    {
                        dis_instr32(
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
                if (is_reghi(src))
                {
                    // lsrs r%dst, r%src, 8
                    dis_instr16(
                        0x0A00
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                }
                else
                {
                    // movs r%dst, r%src
                    dis_instr16(
                        0x0000
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                    
                    // uxtb r%dst, r%dst
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                }
            }
            else if (src == OPARG_A)
            {
                if (is_reghi(dst))
                {
                    // uxtb r%dst, r%dst
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // orr r%dst, r%dst, r%src, lsl #8
                    dis_instr32(
                        0xEA402000
                        | (reg_armidx(dst) << 16)
                        | (reg_armidx(dst) << 8)
                        | (reg_armidx(src) << 0)
                    );
                }
                else
                {
                    // and r%dst, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(dst) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(dst) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                    
                    // orr r%dst, r%src
                    dis_instr16(
                        0x4300
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                }
            }
            else // neither operand is A
            {
                if (reg_armidx(dst) == reg_armidx(src))
                {
                    if (is_reghi(src))
                    {
                        // lsrs r, r, $8
                        dis_instr16(
                            0x0A00
                            | (reg_armidx(dst) << 3)
                            | (reg_armidx(dst) << 0)
                        );
                    }
                    else
                    {
                        // uxtb r, r
                        dis_instr16(
                            0xB2C0
                            | (reg_armidx(dst) << 3)
                            | (reg_armidx(dst) << 0)
                        );
                    }
                    
                    // orr r, r, r, lsl #8
                    dis_instr32(
                        0xEA402000
                        | (reg_armidx(dst) << 16)
                        | (reg_armidx(dst) << 8)
                        | (reg_armidx(src) << 0)
                    );
                }
                else if (is_reghi(dst) && is_reghi(src))
                {
                    // uxtb r%dst, r%dst
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // push {r%src}
                    dis_instr16((1 << reg_armidx(src)) | 0xb400);
                    
                    // and r%src, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(src) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(src) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                    
                    // orr r%dst, r%src
                    dis_instr16(
                        0x4300
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                    
                    // pop {r%src}
                    dis_instr16((1 << reg_armidx(src)) | 0xbc00);
                }
                else if (is_reglo(dst) && is_reglo(src))
                {
                    // and r%dst, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(dst) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(dst) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                    
                    // push {r%src}
                    dis_instr16((1 << reg_armidx(src)) | 0xb400);
                    
                    // uxtb r%src, r%src
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(src) << 3)
                        | (reg_armidx(src) << 0)
                    );
                    
                    // orr r%dst, r%src
                    dis_instr16(
                        0x4300
                        | (reg_armidx(dst) << 0)
                        | (reg_armidx(src) << 3)
                    );
                    
                    // pop {r%src}
                    dis_instr16((1 << reg_armidx(src)) | 0xbc00);
                }
                else if (is_reghi(dst) && !is_reghi(src))
                {
                    
                    // uxtb r%dst, r%dst
                    dis_instr16(
                        0xB2C0
                        | (reg_armidx(dst) << 3)
                        | (reg_armidx(dst) << 0)
                    );
                    
                    // orr r%dst, r%dst, r%src, lsl #8
                    dis_instr32(
                        0xEA402000
                        | (reg_armidx(dst) << 16)
                        | (reg_armidx(dst) << 8)
                        | (reg_armidx(src) << 0)
                    );
                    
                    // OPTIMIZE -- can skip this if we know src's hi is 0.
                    // and r%dst, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(dst) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(dst) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                }
                else if (!is_reghi(dst) && is_reghi(src))
                {
                    // and r%dst, $ff00
                    unsigned imm_encoding = thumbexpand_encoding(0xff, 24);
                    dis_instr32(
                        0xf0000000
                        | (bits(imm_encoding, 11, 1) << 26)
                        | (reg_armidx(dst) << 16)
                        | (bits(imm_encoding, 8, 3) << 12)
                        | (reg_armidx(dst) << 8)
                        | (bits(imm_encoding, 0, 8) << 0)
                    );
                    
                    // orr r%dst, r%dst, r%src, lsr #8
                    dis_instr32(
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
    else if (src == OPARG_BCm || src == OPARG_DEm || src == OPARG_HLm)
    {
        dis.use_r_flex = 1;
        
        // push {...}
        unsigned regs = used_registers() & 0xf & ~(1 << REG_FLEX);
        if (regs) dis_instr16(regs | 0xb400);
        
        // movs r0, r%src
        dis_instr16(0x0000 | (reg_armidx(src) << 3));
        
        // call read
        dis_bl((fn_type)opts.read);
        
        // pop {...}
        if (regs) dis_instr16(regs | 0xbc00);
    }
    else if (is_reg16(dst))
    {
        if (src == OPARG_i16)
        // e.g. ld bc, $1234
        {
            // movw r%dst, imm
            dis_instr32(
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

static void dis_rotate(rotate_type rt, oparg dst)
{
    // TODO
}

static void dis_swap(oparg dst)
{
    // TODO
}

static void dis_jump(oparg condition, oparg addrmode)
{
    // TODO
}

static void dis_call(oparg condition)
{
    // TODO
}

static void dis_rst(unsigned x)
{
    // TODO
}

static void dis_ret(oparg condition)
{
    // TODO
}

static void dis_reti(void)
{
}

// convert to bcd
static void dis_daa(void)
{
}

// set carry flag
static void dis_scf(void)
{
}

// flip carry flag
static void dis_ccf(void)
{
}

// one's complement of A
static void dis_cpl(void)
{
}

static void dis_di(void)
{
}

static void dis_ei(void)
{
}

static void dis_add(oparg dst, oparg src, int carry)
{
}

static void dis_sub(oparg dst, oparg src, int carry)
{
}

static void dis_cp(oparg cmp)
{
}

static void dis_and(oparg src)
{
}

static void dis_or(oparg src)
{
}

static void dis_xor(oparg src)
{
}

static void dis_bit(unsigned b, oparg src)
{
}

// v: 0 if clear, 1 if set.
static void dis_set(unsigned b, oparg src, int v)
{
}

static void dis_push(oparg src)
{
}

static void dis_pop(oparg src)
{
}

static int disassemble_done(void)
{
    if (dis.error) return 1;
    if (dis.rom >= dis.romend - 2) return 1;
    if (dis.armc >= 0x200) return 1;
    return dis.done;
}

static void disassemble_begin(uint32_t gb_rom_offset, uint32_t gb_start_offset, uint32_t gb_end_offset)
{
    memset(&dis, 0, sizeof(dis));
    dis.armcap = 0x100;
    dis.arm = (armop*)malloc(dis.armcap * sizeof(armop));
    if (!dis.arm)
    {
        dis.error = 1;
        return;
    }
    for (size_t i = 0; i < JIT_START_PADDING_ENTRY_C; ++i)
    {
        dis.arm[i] = ARMOP(
            .produce = disp_skip
        );
    }
    dis.use_r_a = 1;
    dis.armc = JIT_START_PADDING_ENTRY_C;
    dis.rom = (const uint8_t*)opts.rom + gb_rom_offset;
    dis.romend = (const uint8_t*)opts.rom + gb_end_offset;
    dis.romstart = (const uint8_t*)opts.rom + gb_start_offset;
}

static void disassemble_capacity(void)
{
    if (dis.armc >= dis.armcap - 0x80)
    {
        dis.armcap *= 2;
        armop* arm = (armop*)realloc(dis.arm, dis.armcap * sizeof(armop));
        if (arm == NULL)
        {
            dis.error = 1;
        }
        else
        {
            dis.arm = arm;
        }
    }
}

static oparg arg_from_op(uint8_t op)
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
    switch (op = dis_read_rom_byte())
    {
        case 0x00: // NOP
            return;
            
        case 0x01:
            return dis_ld(OPARG_BC, OPARG_i16);
            
        case 0x02:
            return dis_ld(OPARG_BCm, OPARG_A);
            
        case 0x03:
            return dis_inc(OPARG_BC);
        
        case 0x04:
            return dis_inc(OPARG_B);
            
        case 0x05:
            return dis_dec(OPARG_B);
            
        case 0x06:
            return dis_ld(OPARG_B, OPARG_i8);
            
        case 0x07:
            return dis_rotate(ROT_LEFT_CARRY, OPARG_A);
            
        case 0x08:
            return dis_ld(OPARG_i16m, OPARG_SP);
            
        case 0x09:
            return dis_add(OPARG_HL, OPARG_BC, 0);
            
        case 0x0A:
            return dis_ld(OPARG_A, OPARG_BCm);
            
        case 0x0B:
            return dis_dec(OPARG_BC);
            
        case 0x0C:
            return dis_inc(OPARG_C);
            
        case 0x0D:
            return dis_dec(OPARG_C);
        
        case 0x0E:
            return dis_ld(OPARG_C, OPARG_i8);
            
        case 0x0F:
            return dis_rotate(ROT_RIGHT_CARRY, OPARG_A);
        
        case 0x10: // STOP
            dis.done = 1;
            return dis_delegate(opts.stop);
            
        case 0x11:
            return dis_ld(OPARG_DE, OPARG_i16);
            
        case 0x12:
            return dis_ld(OPARG_DEm, OPARG_A);
            
        case 0x13:
            return dis_inc(OPARG_DE);
            
        case 0x14:
            return dis_inc(OPARG_D);
            
        case 0x15:
            return dis_dec(OPARG_D);
            
        case 0x16:
            return dis_ld(OPARG_D, OPARG_i8);
            
        case 0x17:
            return dis_rotate(ROT_LEFT, OPARG_A);
            
        case 0x18:
            return dis_jump(COND_none, ADDR_r8);
            
        case 0x19:
            return dis_add(OPARG_HL, OPARG_DE, 0);
            
        case 0x1A:
            return dis_ld(OPARG_A, OPARG_DEm);
            
        case 0x1B:
            return dis_dec(OPARG_DE);
            
        case 0x1C:
            return dis_inc(OPARG_E);
            
        case 0x1D:
            return dis_dec(OPARG_E);
            
        case 0x1E:
            return dis_ld(OPARG_E, OPARG_i8);
            
        case 0x1F:
            return dis_rotate(ROT_RIGHT, OPARG_A);
            
        case 0x20:
            return dis_jump(COND_nz, ADDR_r8);
            
        case 0x21:
            return dis_ld(OPARG_HLmi, OPARG_A);
            
        case 0x22:
            return dis_inc(OPARG_HL);
            
        case 0x23:
            return dis_inc(OPARG_HL);
            
        case 0x24:
            return dis_inc(OPARG_H);
            
        case 0x25:
            return dis_dec(OPARG_H);
            
        case 0x26:
            return dis_ld(OPARG_HLm, OPARG_i8);
            
        case 0x27:
            return dis_daa();
            
        case 0x28:
            return dis_jump(COND_z, ADDR_r8);
            
        case 0x29:
            return dis_add(OPARG_HL, OPARG_HL, 0);
            
        case 0x2A:
            return dis_ld(OPARG_A, OPARG_HLmi);
            
        case 0x2B:
            return dis_dec(OPARG_HL);
            
        case 0x2C:
            return dis_inc(OPARG_L);
            
        case 0x2D:
            return dis_dec(OPARG_L);
            
        case 0x2E:
            return dis_ld(OPARG_L, OPARG_i8);
            
        case 0x2F:
            return dis_cpl();
            
        case 0x30:
            return dis_jump(COND_nc, ADDR_r8);
            
        case 0x31:
            return dis_ld(OPARG_SP, OPARG_i16);
            
        case 0x32:
            return dis_ld(OPARG_HLmd, OPARG_A);
            
        case 0x33:
            return dis_inc(OPARG_SP);
            
        case 0x34:
            return dis_inc(OPARG_HLm);
            
        case 0x35:
            return dis_dec(OPARG_HLm);
            
        case 0x36:
            return dis_ld(OPARG_HL, OPARG_i8);
            
        case 0x37:
            return dis_scf();
            
        case 0x38:
            return dis_jump(COND_c, ADDR_r8);
            
        case 0x39:
            return dis_add(OPARG_HL, OPARG_SP, 0);
            
        case 0x3A:
            return dis_ld(OPARG_A, OPARG_HLmd);
            
        case 0x3B:
            return dis_dec(OPARG_SP);
            
        case 0x3C:
            return dis_inc(OPARG_A);
            
        case 0x3D:
            return dis_dec(OPARG_A);
            
        case 0x3E:
            return dis_ld(OPARG_A, OPARG_i8);
            
        case 0x3F:
            return dis_ccf();
        
        // register transfer
        case 0x40 ... 0x75:
        case 0x77 ... 0x7F:
            switch (op / 0x8)
            {
                case 0x40/8:
                    return dis_ld(OPARG_B, arg_from_op(op));
                case 0x48/8:
                    return dis_ld(OPARG_C, arg_from_op(op));
                case 0x50/8:
                    return dis_ld(OPARG_D, arg_from_op(op));
                case 0x58/8:
                    return dis_ld(OPARG_E, arg_from_op(op));
                case 0x60/8:
                    return dis_ld(OPARG_H, arg_from_op(op));
                case 0x68/8:
                    return dis_ld(OPARG_L, arg_from_op(op));
                case 0x70/8:
                    return dis_ld(OPARG_HLm, arg_from_op(op));
                case 0x78/8:
                    return dis_ld(OPARG_A, arg_from_op(op));
            }
            
        case 0x76: // HALT
            dis.done = 1;
            JIT_DEBUG_MESSAGE("disassembling HALT");
            return dis_delegate(opts.halt);
            
        // arithmetic
        case 0x80 ... 0xBF:
            switch (op)
            {
            case 0x80 ... 0x87:
                return dis_add(OPARG_A, arg_from_op(op), 0);
            case 0x88 ... 0x8F:
                return dis_add(OPARG_A, arg_from_op(op), 1);
            case 0x90 ... 0x97:
                return dis_sub(OPARG_A, arg_from_op(op), 0);
            case 0x98 ... 0x9F:
                return dis_sub(OPARG_A, arg_from_op(op), 1);
            case 0xA0 ... 0xA7:
                return dis_and(arg_from_op(op));
            case 0xA8 ... 0xAF:
                return dis_xor(arg_from_op(op));
            case 0xB0 ... 0xB7:
                return dis_or(arg_from_op(op));
            case 0xB8 ... 0xBF:
                return dis_cp(arg_from_op(op));
            }
        
        case 0xC0:
            return dis_ret(COND_nz);
        
        case 0xC1:
            return dis_pop(OPARG_BC);
            
        case 0xC2:
            return dis_jump(COND_nz, OPARG_i16);
            
        case 0xC3:
            return dis_jump(COND_none, OPARG_i16);
            
        case 0xC4:
            return dis_call(COND_nz);
            
        case 0xC5:
            return dis_push(OPARG_BC);
            
        case 0xC6:
            return dis_add(OPARG_A, OPARG_i8, 0);
            
        case 0xC7:
            return dis_rst(0);
        
        case 0xC8:
            return dis_ret(COND_z);
        
        case 0xC9:
            return dis_ret(COND_none);
            
        case 0xCA:
            return dis_jump(COND_z, OPARG_i16);
            
        case 0xCB:
            {
                uint8_t opx = dis_read_rom_byte();
                oparg arg = arg_from_op(opx);
                switch(opx/8)
                {
                case 0x00/8:
                    return dis_rotate(ROT_LEFT_CARRY, arg);
                case 0x08/8:
                    return dis_rotate(ROT_RIGHT_CARRY, arg);
                case 0x10/8:
                    return dis_rotate(ROT_LEFT, arg);
                case 0x18/8:
                    return dis_rotate(ROT_RIGHT, arg);
                case 0x20/8:
                    return dis_rotate(SHIFT_LEFT, arg);
                case 0x28/8:
                    return dis_rotate(SHIFT_RIGHT, arg);
                case 0x30/8:
                    return dis_swap(arg);
                case 0x38/8:
                    return dis_rotate(SHIFT_RIGHT_L, arg);
                case 0x40/8 ... 0x78/8:
                    return dis_bit((opx/8 - 0x40/8), arg);
                case 0x80/8 ... 0xB8/8:
                    return dis_set((opx/8 - 0x80/8), arg, 0);
                case 0xC0/8 ... 0xF8/8:
                    return dis_set((opx/8 - 0xC0/8), arg, 1);
                }
            }
            
        case 0xCC:
            return dis_call(COND_z);
            
        case 0xCD:
            return dis_call(COND_none);
            
        case 0xCE:
            return dis_add(OPARG_A, OPARG_i8, 1);
            
        case 0xCF:
            return dis_rst(1);
            
        case 0xD0:
            return dis_ret(COND_nc);
        
        case 0xD1:
            return dis_pop(OPARG_DE);
            
        case 0xD2:
            return dis_jump(COND_nc, OPARG_i16);
            
        case 0xD4:
            return dis_call(COND_nc);
            
        case 0xD5:
            return dis_push(OPARG_DE);
            
        case 0xD6:
            return dis_sub(OPARG_A, OPARG_i8, 0);
            
        case 0xD7:
            return dis_rst(2);
        
        case 0xD8:
            return dis_ret(COND_c);
        
        case 0xD9:
            return dis_reti();
            
        case 0xDA:
            return dis_jump(COND_c, OPARG_i16);
            
        case 0xDC:
            return dis_call(COND_c);
            
        case 0xDE:
            return dis_sub(OPARG_A, OPARG_i8, 1);
            
        case 0xDF:
            return dis_rst(3);
            
        case 0xE0:
            return dis_ld(OPARG_i8m, OPARG_A);
            
        case 0xE1:
            return dis_pop(OPARG_HL);
            
        case 0xE2:
            return dis_ld(OPARG_Cm, OPARG_A);
            
        case 0xE5:
            return dis_push(OPARG_HL);
            
        case 0xE6:
            return dis_and(OPARG_i8);
            
        case 0xE7:
            return dis_rst(4);
            
        case 0xE8:
            return dis_add(OPARG_SP, OPARG_i8, 0);
            
        case 0xE9:
            return dis_jump(COND_none, ADDR_HL);
            
        case 0xEA:
            return dis_ld(OPARG_i16m, OPARG_A);
            
        case 0xED:
            return dis_xor(OPARG_i8);
            
        case 0xEF:
            return dis_rst(5);
            
        case 0xF0:
            return dis_ld(OPARG_A, OPARG_i8m);
            
        case 0xF1:
            return dis_pop(OPARG_AF);
            
        case 0xF2:
            return dis_ld(OPARG_A, OPARG_Cm);
            
        case 0xF3:
            return dis_di();
            
        case 0xF5:
            return dis_push(OPARG_AF);
            
        case 0xF6:
            return dis_or(OPARG_i8);
            
        case 0xF7:
            return dis_rst(6);
            
        case 0xF8:
            return dis_ld(OPARG_HL, OPARG_i8sp);
            
        case 0xF9:
            return dis_ld(OPARG_SP, OPARG_HL);
            
        case 0xFA:
            return dis_ld(OPARG_A, OPARG_i16m);
            
        case 0xFB:
            return dis_ei();
            
        case 0xFD:
            return dis_cp(OPARG_i8);
            
        case 0xFF:
            return dis_rst(7);
            
        default:
            dis.done = 1;
            dis_delegate(opts.illegal);
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
    
    #define epilogue_16(x) dis_instr16(x)
    #define epilogue_32(x) dis_instr32(x)
    #define prologue_16(x) do { dis.arm[padc++] = ARMOP(.args=(void*)(uintptr_t)(x), .produce=disp_16); } while (0)
    #define prologue_32(x) do { dis.arm[padc++] = ARMOP(.args=(void*)(uintptr_t)(x), .produce=disp_32);  } while (0)
    
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
        JIT_DEBUG_MESSAGE("r1 <- regfile: %8x", (uintptr_t)(void*)opts.regs);
        
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
        
        if (dis.use_r_flex)
        {
            // regfile is flex register, so we must store this for later.
            prologue_16(0xb400 | REG_FLEX);
        }
    }
    
    if (dis.init_r_a)
    {
        prologue_16(
            0x6800
            | (offsetof(jit_regfile_t, a) << 6)
            | (REG_REGF << 3)
            | (REG_A << 0)
        );
    }
    
    if (dis.init_r_bc)
    {
        prologue_16(
            0x8800
            | (offsetof(jit_regfile_t, bc) << 5)
            | (REG_REGF << 3)
            | (REG_BC << 0)
        );
    }
    
    if (dis.init_r_de)
    {
        prologue_16(
            0x8800
            | (offsetof(jit_regfile_t, de) << 5)
            | (REG_REGF << 3)
            | (REG_DE << 0)
        );
    }
    
    if (dis.init_r_hl)
    {
        prologue_16(
            0x8800
            | (offsetof(jit_regfile_t, hl) << 5)
            | (REG_REGF << 3)
            | (REG_HL << 0)
        );
    }
    
    if (dis.init_r_sp)
    {
        prologue_16(
            0x8800
            | (offsetof(jit_regfile_t, sp) << 5)
            | (REG_REGF << 3)
            | (REG_SP << 0)
        );
    }
    
    if (dis.use_r_regfile && dis.use_r_flex)
    {
        // regfile is flex register, so we must retrieve it as it's been clobbered
        epilogue_16(0xbc00 | REG_FLEX);
    }
    
    if (dis.dirty_r_a)
    {
        // write value of A back to regfile
        // strb r%a, [r%regf, offsetof(jit_regfile, a)]
        epilogue_16(
            0x7000
            | (offsetof(jit_regfile_t, a) << 6)
            | (REG_REGF << 3)
            | (REG_A << 0)
        );
    }
    
    if (dis.dirty_r_bc)
    {
        // strh r%bc, [r%regf, offsetof(jit_regfile, bc)]
        epilogue_16(
            0x8000
            | ((offsetof(jit_regfile_t, bc)/2) << 6)
            | (REG_REGF << 3)
            | (REG_BC << 0)
        );
    }
    
    if (dis.dirty_r_de)
    {
        epilogue_16(
            0x8000
            | ((offsetof(jit_regfile_t, de)/2) << 6)
            | (REG_REGF << 3)
            | (REG_DE << 0)
        );
    }
    
    if (dis.dirty_r_hl)
    {
        epilogue_16(
            0x8000
            | ((offsetof(jit_regfile_t, hl)/2) << 6)
            | (REG_REGF << 3)
            | (REG_HL << 0)
        );
    }
    
    if (dis.dirty_r_sp)
    {
        epilogue_16(
            0x8000
            | ((offsetof(jit_regfile_t, sp)/2) << 6)
            | (REG_REGF << 3)
            | (REG_SP << 0)
        );
    }
    
    // pop {...}
    if (reg_push && !dis.use_lr)
    {
        epilogue_16(0xbc00 | reg_push);
    }
    else if (reg_push && dis.use_lr)
    {
        epilogue_32(reg_push | 0xE8BD4000);
    }
    else if (dis.use_lr)
    {
        epilogue_32(0xF85DEB04);
    }
    
    // return.
    epilogue_16(0x4770);
    
    assert(padc <= JIT_START_PADDING_ENTRY_C);
}

static void* disassemble_end(void)
{
    if (dis.error)
    {
        if (dis.arm) free(dis.arm);
        return NULL;
    }
    
    disassemble_padding();
    
    // accumulate size of output
    
    size_t outsize = 0;
    for (size_t i = 0; i < dis.armc; ++i)
    {
        uint8_t size = dis.arm[i].produce(dis.arm[i].args, NULL);
        #ifdef JIT_DEBUG
        dis.arm[i].length = size;
        #endif
        outsize += size;
    }
    
    // allocate output buffer
    uint16_t* const out = (uint16_t*)malloc(outsize * sizeof(uint16_t));
    if (!out)
    {
        dis.error = 1;
        if (dis.arm) free(dis.arm);
        return NULL;
    }
    
    JIT_DEBUG_MESSAGE("base address: %8x", out);
    uint16_t* outb = out;
    
    for (size_t i = 0; i < dis.armc; ++i)
    {
        outb += dis.arm[i].produce(dis.arm[i].args, outb);
    }
    
    #ifdef JIT_DEBUG
    #define printf opts.playdate->system->logToConsole
    JIT_DEBUG_MESSAGE("arm code: ");
    const char* outmsg;
    size_t armi = 0;
    size_t armseek = 0;
    printf("Used: %02x; Dirty: %02x", used_registers(), dirty_registers());
    for (const uint16_t* arm = out; arm && arm < out+outsize;)
    {
        // zip along with arm disp as well
        while (armseek < arm - out || (dis.arm[armi].length == 0 && armseek < outsize))
        {
            armseek += dis.arm[armi++].length;
        }
        
        if (armseek == arm - out)
        {
            if (dis.arm[armi].romsrc)
            {
                printf("; z80: %02X", *dis.arm[armi].romsrc);
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
    
    free(dis.arm);
    
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
        size_t armi_prev = dis.armc;
        const uint8_t* romsrc = dis.rom;
        #endif
        
        disassemble_instruction();
        
        #ifdef JIT_DEBUG
        if (dis.armc > armi_prev)
        {
            dis.arm[armi_prev].romsrc = romsrc;
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

