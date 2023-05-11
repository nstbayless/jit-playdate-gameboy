#pragma once

#include "pd_api.h"
#include <stdint.h>
#include <stdbool.h>

#define JIT_ZNH_BIT_N 5
typedef struct jit_regfile_t
{
    // we store as 32-bit instead of 16 for better alignment properties.
    uint32_t a;
    uint32_t bc;
    uint32_t de;
    uint32_t hl;
    uint32_t sp;
    
    // bit 0 is 1 iff carry is set
    uint32_t carry;
    
    // 0 if set
    // any nonzero value if unset
    uint32_t z;
    
    // bit 0: garbage (could be 0 or 1)
    // bit 5: n
    // byte 1: c (in addition to 0 and 1, can also be 0x10; if c is 0x10 and a and b are 0, then h is set.)
    // byte 2: operand b
    // byte 3: operand a
    // h is implicit: it is 1 if (a&0xf+b&xf+c)&0x10
    uint32_t nh;

    uint32_t pc;
    uint32_t ime;
} jit_regfile_t;

static inline bool jit_regfile_getn(uint32_t nh)
{
    return (nh >> 5) & 1;
}

static inline bool jit_regfile_geth(uint32_t nh)
{
    uint8_t c = (znh >> 8) & 0xff;
    uint8_t b = (znh >> 16) & 0x0f;
    uint8_t a = (znh >> 24) & 0x0f;
    int n = jit_regfile_getn(nh);
    if (!n)
    {
        return !!((a + b + c) & 0x10);
    }
    else
    {
        return !!((a - b - c) & 0x10);
    }
}

// returns corrected value in lower 8 bits (r0)
// returns new value of carry in bit 32 (r1)
// all other bits are clear.
// remember to set Z=* and H=0 after calling this.
static inline uint64_t jit_regfile_daa(const uint32_t nh, uint8_t v, const bool c)
{
    // adapted with reference to peanut_gb.h
    const uint8_t n = jit_regfile_getn(nh);
    const uint8_t h = jit_regfile_geth(nh);
    if (n)
    {
        if (h)
        {
            v = (v - 0x06) & 0xFF;
        }

        if (c)
        {
            v -= 0x60;
        }
    }
    else
    {
        if (h || (v & 0x0F) > 9)
        {
            v += 0x06;
        }

        if (c || v > 0x9F)
        {
            v += 0x60;
        }
    }

    if (v & 0x100)
    {
        c = 1;
    }
    
    return (v) | ((uint64_t)c << 32)
}

static inline uint8_t jit_regfile_get_f(uint32_t carry, uint32_t z, uint32_t nh)
{
    return ((!z) << 7) | (carry << 4) | jit_regfile_geth(nh) << 5 | (jit_regfile_getn(nh) << 6);
}

// can assign the result to the .nh field.
static inline uint32_t jit_regfile_setnh(bool n, bool h)
{
    return (n << 5) | (h << 12);
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