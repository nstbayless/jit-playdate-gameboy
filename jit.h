#pragma once

#include "pd_api.h"
#include <stdint.h>

typedef void (*jit_fn)(void);

typedef struct
{
    // we store as 32-bit instead of 16 for better alignment properties.
    uint32_t a;
    uint32_t hl;
    uint32_t bc;
    uint32_t de;
    uint32_t sp;
    uint32_t pc;
} jit_regfile_t;

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
    jit_regfile_t* regs;
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
#endif