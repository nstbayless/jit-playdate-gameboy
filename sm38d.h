#pragma once
// disassemble sm38 (gameboy)

#include <stdint.h>

// disassembles one instruction (will not exceed sm38d_end)
// assumption: at least three bytes following sm38d are safe to read.
// returns instruction length
int sm38d(const uint8_t* sm38, const char** outchar);