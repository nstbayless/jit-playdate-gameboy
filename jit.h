#pragma once

#include <stdint.h>

typedef void (*jit_fn)(void);

typedef struct
{
    const void* rom;
    const void (*stop)(void);
    const void (*halt)(void);
    const void (*illegal)(void);
    uint16_t* reg_af; // F: ZNHC0000
    uint16_t* reg_bc;
    uint16_t* reg_de;
    uint16_t* reg_hl;
    uint16_t* reg_sp;
    uint16_t* reg_pc;
} jit_opts;

void jit_init(jit_opts opts);
void jit_cleanup(void);

// call this every frame at start of frame.
void jit_memfix(void);

// returns NULL in case of error.
jit_fn jit_get(uint16_t gb_addr, uint16_t gb_bank);