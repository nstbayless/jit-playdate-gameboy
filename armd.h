// disassemble arm

#include <stdint.h>

// assumption: arm & 1 == 0, but refers to thumb code anyway.
const uint16_t* armd(const uint16_t* arm, const char** outchar);