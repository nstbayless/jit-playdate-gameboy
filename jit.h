#pragma once

#include "pd_api.h"
#include <stdint.h>

typedef void (*jit_fn)(void);

typedef struct
{
    uint16_t af, bc, de, hl, sp, pc;
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

// returns NULL in case of error.
jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank);