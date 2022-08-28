#include "jit.h"

// NOTE: mem.h may be missing. It is not checked into version control.
// it contains two functions: memfix() and flush_icache()
// Please ask author for this file, or implement it yourself.
#ifdef TARGET_PLAYDATE
    #include "mem.h"
#endif

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define JIT_HASHTABLE_SIZE 0x4000 // must be power of 2, at most 0x8000

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

static jit_fn jit_add_ht_entry(uint16_t gb_addr, uint16_t gb_bank, jit_fn fn)
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

void jit_init(jit_opts _opts)
{
    memset(jit_hashtable, 0, sizeof(jit_hashtable));
    opts = _opts;
}

void jit_cleanup(void)
{
    for (size_t i = 0; i < JIT_HASHTABLE_SIZE; ++i)
    {
        if (jit_hashtable[i])
        {
            for (jit_ht_entry* e = jit_hashtable[i]; e->fn; ++e)
            {
                free((void*)e->fn);
            }
            free(jit_hashtable[i]);
        }
    }
    memset(&opts, 0, sizeof(opts));
}

void jit_memfix(void)
{
    #ifdef TARGET_PLAYDATE
    memfix();
    #endif
}

// disassembly state.
static struct {
    uint16_t* arm;
    uint16_t armc;
    uint16_t armcap;
    const uint8_t* rom;
    const uint8_t* romend;
    unsigned done : 1;
} dis;

// call function directly.
static void dis_delegate(const void (*fn)(void))
{
}

static int dis_set_n()
{
    // TODO
}

static int dis_clear_n()
{
    // TODO
}

static void dis_ld(oparg dst, oparg src)
{
    // TODO
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

static void dis_reti()
{
}

// convert to bcd
static void dis_daa()
{
}

// set carry flag
static void dis_scf()
{
}

// flip carry flag
static void dis_ccf()
{
}

// one's complement of A
static void dis_cpl()
{
}

static void dis_di()
{
}

static void dis_ei()
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

static int disassemble_done()
{
    if (dis.rom >= dis.romend - 2) return 1;
    return dis.done;
}

static void disassemble_begin(uint32_t gb_rom_offset, uint32_t gb_end_offset)
{
    dis.armcap = 0x100;
    dis.done = 0;
    dis.arm = (uint16_t*)malloc(dis.armcap * sizeof(uint16_t));
    dis.armc = 0;
    dis.rom = (const uint8_t*)opts.rom + gb_rom_offset;
    dis.romend = (const uint8_t*)opts.rom + gb_end_offset;
}

static void disassemble_capacity()
{
    if (dis.armc >= dis.armcap / 2)
    {
        dis.armcap *= 2;
        dis.arm = (uint16_t*)realloc(dis.arm, dis.armcap * sizeof(uint16_t));
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
}

static void disassemble_instruction()
{
    disassemble_capacity();
    uint8_t op;
    switch (op = *++dis.rom)
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
            return dis_delegate(opts.halt);
            
        // arithmetic
        case 0x80 ... 0xBF:
            switch (op / 0x8)
            {
            case 0x80/8:
                return dis_add(OPARG_A, arg_from_op(op), 0);
            case 0x88/8:
                return dis_add(OPARG_A, arg_from_op(op), 1);
            case 0x90/8:
                return dis_sub(OPARG_A, arg_from_op(op), 0);
            case 0x98/8:
                return dis_sub(OPARG_A, arg_from_op(op), 1);
            case 0xA0/8:
                return dis_and(arg_from_op(op));
            case 0xA8/8:
                return dis_xor(arg_from_op(op));
            case 0xB0/8:
                return dis_or(arg_from_op(op));
            case 0xB8/8:
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
                uint8_t opx = *++dis.rom;
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

static void* disassemble_end()
{
    size_t pad_begin = 0;
    size_t pad_end = 1;
    
    uint16_t* out = (uint16_t*)malloc((dis.armc + pad_begin + pad_end) * sizeof(uint16_t));
    memcpy(out + pad_begin, dis.arm, dis.armc * sizeof(uint16_t));
    
    // append return instruction
    out[dis.armc + pad_begin] = 0x4770;
    return out;
}

static jit_fn jit_compile(uint16_t gb_addr, uint16_t gb_bank)
{
    if (gb_addr >= 0x8000) return NULL;
    if (gb_addr == 0x7FFF || gb_addr == 0x3FFF || gb_addr == 0x7FFE || gb_addr == 0x3FFE) return NULL;
    
    uint32_t gb_rom_offset = (gb_addr % 0x4000) | ((uint32_t)gb_bank * 0x4000);
    uint32_t gb_end_offset = (((uint32_t)gb_bank + 1) * 0x4000);
    
    disassemble_begin(gb_rom_offset, gb_end_offset);
    
    while (!disassemble_done())
    // disassemble each gameboy instruction one at a time.
    {
        disassemble_instruction();
    }
}

jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank)
{
    jit_fn f = get_jit_fn(gb_addr, gb_bank);
    if (f) return f;
    
    // no existing fn
    return jit_compile(gb_addr, gb_bank);
}
