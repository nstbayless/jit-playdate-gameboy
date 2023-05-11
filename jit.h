#pragma once

#include "pd_api.h"
#include <stdint.h>


#define JIT_ZNH_BIT_N 5
typedef struct jit_regfile_t
{
    // we store as 32-bit instead of 16 for better alignment properties.
    uint32_t a;
    uint32_t bc;
    uint32_t de;
    uint32_t hl;
    uint32_t sp;
    uint32_t carry;
    
    // 0 if set
    // any nonzero value if unset
    uint32_t z;
    
    // bit 0: garbage (could be 0 or 1)
    // bit 5: n
    // byte 1: c
    // byte 2: operand a
    // byte 3: operand b
    // h is implicit: it is 1 if (a&0xf+b&xf+c)&0x10
    uint32_t nh;

    uint32_t pc;
} jit_regfile_t;

static inline int jit_regfile_getn(uint32_t znh)
{
    return (znh >> 5) & 1;
}

static inline int jit_regfile_geth(uint32_t znh)
{
    uint8_t c = (znh >> 8) & 0xff;
    uint8_t a = (znh >> 16) & 0x0f;
    uint8_t b = (znh >> 24) & 0x0f;
    
    // OPTIMIZE: shift to php z register...
    return !!((a + b + c) & 0x10);
}

typedef void (*jit_fn)(jit_regfile_t*);

typedef struct
{
    const void* rom;
    void* wram;
    void* hram;
    void (*stop)(void);
    void (*halt)(void);
    void (*illegal)(void);
    uint8_t (*read)(uint16_t addr);
    void (*write)(uint16_t addr, uint8_t value);
    uint16_t (*readword)(uint16_t addr);
    void (*writeword)(uint16_t addr, uint16_t value);
    jit_regfile_t* regs;
    unsigned fixed_bank : 1; // 1 if the bank the code is in is switchable. (if no mapper, this should be 0.)
    unsigned is_gb_color : 1;
    PlaydateAPI* playdate;
} jit_opts;

void jit_init(jit_opts opts);
void jit_cleanup(void);

// call this at least once every frame at start of frame.
void jit_memfix(void);

void jit_invalidate_cache(void);

// returns NULL in case of error.
jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank);

// FIXME -- swap out for NDEBUG
#ifndef __NDEBUG_
    #define jit_assert(_A) do {if (!(_A)) playdate->system->error("assertion failed: " #_A);} while (0)
    #define jit_assert_pd(_A, _PD) do {if (!(_A)) (_PD)->system->error("assertion failed: " #_A);} while (0)
#else
    #define jit_assert(...) {}
    #define jit_assert_pd(...) {}
#endif