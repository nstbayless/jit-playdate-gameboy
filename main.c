#ifndef TARGET_QEMU
    #include <pd_api.h>
    #include "pdnewlib.h"
    #define printf playdate->system->logToConsole
#else
    #include <stdio.h>
#endif

#include <stdarg.h>

#include "jit.h"
#include <stdint.h>

#ifdef TARGET_QEMU
static void __attribute__ ((noinline)) __semihost(unsigned int swi, const void* arg)
{
    asm("bkpt 0xab");
}

void _printstr(const char* s)
{
    __semihost(4, s);
}

void _printf(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    int length = vsnprintf(NULL, 0, fmt, args);

    va_end(args);

    if (length > 0)
    {
        char buffer[length+2];

        va_start(args, fmt);

        vsnprintf(buffer, length + 1, fmt, args);

        va_end(args);

        _printstr(buffer);
    }
}

#define printf _printf

extern unsigned long __stack;
void Reset_Handler(void) __attribute__((naked));
void Default_Handler(void) __attribute__((interrupt));
extern void __attribute__((noreturn)) _start(void);

// Define the interrupt vector table
__attribute__((section(".isr_vector")))
void (*const g_pfnVectors[])(void) = {
    (void (*)(void))((unsigned long)&__stack),  // Initial stack pointer
    _start,                                        // Reset Handler
    Default_Handler,                               // Non-maskable Interrupt (NMI)
    Default_Handler,                               // Hard Fault
    // Add other interrupt handlers here as needed
};

void Default_Handler(void)
{
    while (1);
}


#endif

static volatile int b = 0;
#ifndef TARGET_QEMU
static PlaydateAPI* playdate;
#endif
static uint8_t wram[0x2000];
static uint8_t hram[0x7F];

static jit_regfile_t regs;
static unsigned halt;

#include "tests/test_12.c"
#include "tests/test_1.c"

static const uint8_t gbrom_nop[] = {
    0x10,     // stop
};

static const uint8_t gbrom_ld[] = {
    0x7C,               // ld a, h
    0x3E, 0x69,         // ld a, $69
    0x06, 0x3A,         // ld b, $3A
    0x11, 0x34, 0x12,   // ld de, $1234
    0x10,               // stop
};

static const uint8_t gbrom_transfer[] = {
    0x16, 0x20,         // ld d, $20
    0x5A,               // ld e, d
    0x42,               // ld b, d
    0x4B,               // ld c, e
    0x61,               // ld h, c
    0x68,               // ld l, b
    0x1E, 0x33,         // ld e, $33
    0x53,               // ld d, e
    0x10,               // stop
};

static const uint8_t gbrom_memaccess[] = {
    0x3E, 0x30,         // ld a, $30
    0xEA, 0x00, 0xC0,   // ld ($C000), a
    0x3E, 0x10,         // ld a, $10
    0xFA, 0x00, 0xC0,   // ld a, ($C000)
    0x10,               // stop
};

static const uint8_t gbrom_bitmath_inc[] = {
    0x21, 0x00, 0xC0,   // ld hl, $C000
    0x3E, 0x85,         // ld a, $85
    0xB6,               // or (hl)
    0x47,               // ld b, a
    0x34,               // inc (hl)
    0xEE, 0x90,         // xor a, $90
    0x4f,               // ld c, a
    0x03,               // inc bc
    0x0c,               // inc c
    0x04,               // inc b
    0xAF,               // xor a, a
    0x10,               // stop
};

static const uint8_t gbrom_rotate[] = {
    0x97,       // sub a    | a = 0
    0x5F,       // ld e, a  | e = 0
    0x57,       // ld d, a  | d = 0
    0x3F,       // ccf
    0x17,       // rla      | a = 1
    0x07,       // rlca     | a = 2
    0x4f,       // ld c, a  | c = 2
    0x37,       // scf
    0x1f,       // rra      | a = 81
    0x47,       // ld b, a  | b = 81
    0x07,       // rlca     | a = 03
    0xcb, 0x37, // swap a   | a = 30
    0xcb, 0x19, // rr c     | c = 1
    0x61,       // ld h, c  | h = 1
    0xcb, 0x19, // rr c     | c = 0
    0x1f,       // rra      | a = 98
    0x37,       // scf
    0xcb, 0x1A, // rr d     | d = 80
    0xcb, 0x1A, // rr d     | d = 40
    0xcb, 0x12, // rl d     | d = 80
    0xcb, 0x3a, // srl d    | d = C0
    0x6A,       // ld l, d  | l = C0
    0xcb, 0x3a, // srl d    | d = E0
    0xcb, 0x1A, // rr d     | d = 70
    0xcb, 0x3a, // srl d    | d = 38
    0xcb, 0x32, // swap d   | d = 83
    0xcb, 0x35, // swap l   | l = 0C
    0xcb, 0x10, // rl b
    0xcb, 0x18, // rr b
    0xcb, 0x11, // rl c
    0xcb, 0x19, // rr c
    0x17,       // rla
    0xcb, 0x1f, // rr a
    0xcb, 0x17, // rl a
    0x1f,       // rra
    0x10,       // stop
};

static const uint8_t gbrom_stack[] = {
    0x31, 0xff, 0xff, // ld sp, $FFFF
    0xf8, 0xf1,       // ld hl, sp-$F
    0xE5,             // push hl
    0x21, 0x12, 0x34, // ld hl, $1043
    0xE5,             // push hl
    0xC1,             // pop bc
    0xE1,             // pop hl
    0xF0, 0xFE,       // ld a, ($FE) | a <- FF
    0x4F,             // ld c, a
    0x0D,             // dec c
    0x5F,             // ld e, a
    0x57,             // ld d, a
    0x9D,             // sbc a, l | a <- F
    0xE2,             // ld (C), a
    0x10,             // stop
};

static const uint8_t gbrom_jump[] = {
    0xAF,             // xor a
    0x47,             // ld b, a
    0x4F,             // ld c, a
    0x67,             // ld h, a
    0x6F,             // ld l, a
    0x18, 0x04,       // jr +4
    0x21, 0x21, 0x21, // ld hl, $2121
    0x10,             // stop
    0xC3, 0x07, 0x00, // jmp $0007
    0x01, 0x11, 0x11, // ld bc, $1111
    0x10              // stop
};

static void test_stop(void)
{
    halt = 1;
    printf("STOP\n");
}

static void test_halt(void)
{
    printf("HALT\n");
}

static void test_illegal(void)
{
    halt = 1;
    printf("ILLEGAL\n");
}

const uint8_t* current_rom=NULL;

static uint8_t test_read(uint16_t addr)
{
    if (addr < 0xC000)
    {
        // XXX
        return current_rom[addr];
    }
    if (addr >= 0xC000 && addr < 0xE000)
    {
        return wram[addr - 0xC000];
    }
    if (addr >= 0xFF80 && addr < 0xFFFF)
    {
        return hram[addr - 0xFF80];
    }
    return 0;
}

static void test_write(uint16_t addr, uint8_t value)
{
    if (addr >= 0xC000 && addr < 0xE000)
    {
        wram[addr - 0xC000] = value;
    }
    if (addr >= 0xFF80 && addr < 0xFFFF)
    {
        hram[addr - 0xFF80] = value;
    }
}

static uint16_t test_read_word(uint16_t addr)
{
    uint16_t value = test_read(addr) | (test_read(addr + 1) << 8);
    //printf("read word: %04X->%04X\n", (unsigned int) addr, (unsigned int)value);
    return value;
}

static void test_write_word(uint16_t addr, uint16_t value)
{
    //printf("write word: %04X<-%04X\n", (unsigned int)addr, (unsigned int)value);
    test_write(addr, value & 0xff);
    test_write(addr+1, value >> 8);
}

static void spin(void)
{
    for (size_t i = 0; i < 0x40000; ++i) asm("nop");
}

static bool do_print = true;

typedef void (*cb_t)(jit_opts*);

void do_test(const uint8_t* rom, cb_t cb)
{
    current_rom = rom;
    jit_opts opts = {
        .rom = rom,
        .pc = 0,
        .wram = wram,
        .hram = hram,
        .stop = test_stop,
        .halt = test_halt,
        .illegal = test_illegal,
        .read = test_read,
        .write = test_write,
        .readword = test_read_word,
        .writeword = test_write_word,
        .ld_b_b = NULL,
        .regs = &regs,
        .fixed_bank = 1,
        .is_gb_color = 0,
        #ifndef TARGET_QEMU
        .playdate = playdate
        #endif
    };
    regs.pc = 0;
    regs.sp = 0xfffe;
    halt = 0;
    
    printf("Initialize jit.\n");
    spin();
    jit_init(opts);
    
    #ifdef __arm__
    while (!halt)
    {
        bool _print = do_print;
        if (cb) cb(&opts);
        
        if (_print) printf("Getting jit at %x.\n", regs.pc);
        if (_print) spin();
        jit_fn fn = jit_get(regs.pc, 0);
        if (_print) printf("Done.\n");
        if (_print) spin();
        if (fn)
        {
            if (regs.pc == 0x4C1)
            {
                printf("de %x\n", regs.de);
            }
            if (_print) printf("invoking %04x @0x%p.\n", regs.pc, (void*)fn);
            if (_print) spin();
            fn(&regs);
            if (_print) printf("Done.\n");
            if (_print) spin();
        }
        else
        {
            printf("jit failed.\n");
            break;
        }
    }
    #else
    (void)jit_get(regs.pc, 0);
    #endif
    printf("Test complete.\n");
    spin();
    jit_cleanup();
}

int update(void* A)
{
    jit_memfix();
    return 0;
} 

void printregs(void)
{
    printf("register a:  %2X\n", regs.a);
    printf("register bc: %4X\n", regs.bc);
    printf("register de: %4X\n", regs.de);
    printf("register hl: %4X\n", regs.hl);
    printf("register sp: %4X\n", regs.sp);
    printf("carry:       %2X\n", regs.carry);
    printf("status:      %2X\n", (int)jit_regfile_p_get_f(&regs));
}

void blargg_print_checksum(void)
{
    printf("checksum: %x%x%x%x\n", hram[0], hram[1], hram[2], hram[3]);
}

void blargg_print_cb(jit_opts* opts)
{
    if (regs.pc == 0x80)
    // see 'jit_test.s'
    {
        char s[2] = {regs.a, 0x0};
        printf("%s", s);
        //printf("  o:%x\n", regs.a);
    }
    if (regs.pc == 0x2C4)
    // print_str_hl
    {
        //printf("print_str_hl~\n");
        //printf("hl: %x\n", regs.hl);
        //printf(  "[0]:%x\n", test_read(regs.hl));
    }
    if (regs.pc == 0x2C7)
    {
        //printf("1-hl: %x\n", regs.hl);
        //printf("   a: %x\n", regs.a);
    }
    if (regs.pc == 0x2CA)
    {
        //printf("0-hl: %x\n", regs.hl);
    }
    if (regs.pc == 0x4B2)
    {
        printf("de: %x\n", regs.de);
    }
    if (regs.pc == 0x4C7)
    {
        blargg_print_checksum();
    }
}

#ifdef TARGET_QEMU
int main(int argc, char** argv)
{
    {
#else
int eventHandler
(PlaydateAPI* pd, PDSystemEvent event, uint32_t arg)
{
    eventHandler_pdnewlib(playdate, event, arg);
    playdate = pd;
    playdate->system->setUpdateCallback(update, NULL);
    
    if (event == kEventInit)
    {
        playdate->system->setAutoLockDisabled(0);
#endif

        #if 0
        printf("Nop:\n");
        do_test(gbrom_nop, NULL);
        
        printf("Ld:\n");
        do_test(gbrom_ld, NULL);
        
        #ifdef __arm__
        printf("register a:  %2X\n", regs.a);
        printf("register bc: %4X\n", regs.bc);
        printf("register de: %4X\n", regs.de);
        printf("register hl: %4X\n", regs.hl);
        jit_assert(regs.a == 0x69);
        jit_assert(regs.bc == 0x3A00);
        jit_assert(regs.de == 0x1234);
        #endif
        
        printf("Transfer:\n");
        do_test(gbrom_transfer, NULL);
        #ifdef __arm__
        printf("register a:  %2X\n", regs.a);
        printf("register bc: %4X\n", regs.bc);
        printf("register de: %4X\n", regs.de);
        printf("register hl: %4X\n", regs.hl);
        
        jit_assert(regs.a == 0x69); // from previous test
        jit_assert(regs.bc == 0x2020);
        jit_assert(regs.de == 0x3333);
        jit_assert(regs.hl == 0x2020);
        #endif
        
        printf("Mem Access:\n");
        do_test(gbrom_memaccess, NULL);
        #ifdef __arm__
        printf("register a:  %2X\n", regs.a);
        printf("register bc: %4X\n", regs.bc);
        printf("register de: %4X\n", regs.de);
        printf("register hl: %4X\n", regs.hl);
        
        jit_assert(regs.a == 0x30);
        jit_assert(wram[0] == 0x30);
        #endif
        
        printf("Bit Math:\n");
        do_test(gbrom_bitmath_inc, NULL);
        #ifdef __arm__
        printf("register a:  %2X\n", regs.a);
        printf("register bc: %4X\n", regs.bc);
        printf("register de: %4X\n", regs.de);
        printf("register hl: %4X\n", regs.hl);
        
        jit_assert(regs.a == 0x0);
        jit_assert(regs.bc == 0xB627);
        jit_assert(regs.z == 0);
        jit_assert(wram[0] == 0x31);
        #endif
        
        printf("Rotate:\n");
        do_test(gbrom_rotate, NULL);
        #ifdef __arm__
        printf("register a:  %2X\n", regs.a);
        printf("register bc: %4X\n", regs.bc);
        printf("register de: %4X\n", regs.de);
        printf("register hl: %4X\n", regs.hl);
        printf("register f:  %2X\n", (int)jit_regfile_p_get_f(&regs));
        
        jit_assert(regs.a == 0x98);
        jit_assert(regs.bc == 0x8100);
        jit_assert(regs.de == 0x8300);
        jit_assert(regs.hl == 0x010C);
        #endif
        
        printf("Stack:\n");
        do_test(gbrom_stack, NULL);
        #ifdef __arm__
        printf("hram[7d-7e]: %2X\n", test_read_word(0xfffd));
        printf("hram[7b-7c]: %2X\n", test_read_word(0xfffb));
        
        jit_assert(regs.a  == 0x0E);
        jit_assert(regs.bc == 0x34FE);
        jit_assert(regs.de == 0xFFFF);
        jit_assert(regs.hl == 0xFFF0);
        jit_assert(regs.sp == 0xFFFF);
        jit_assert(hram[0x7E] == 0x0E);
        jit_assert(hram[0x7D] == 0xF0);
        jit_assert(hram[0x7C] == 0x34);
        jit_assert(hram[0x7B] == 0x12);
        #endif
        
        printf("Jump:\n");
        do_test(gbrom_jump, NULL);
        #ifdef __arm__
        printregs();
        jit_assert(regs.a  == 0x0);
        jit_assert(regs.hl == 0x2121);
        jit_assert(regs.bc == 0x0000);
        #endif
        #endif
        
        #if 0
        printf("test_basic:\n");
        do_test(test_12_gb, blargg_print_cb);
        jit_assert(regs.a  == 0x0);
        #endif
        
        #if 1
        printf("test_special:\n");
        do_print = false;
        do_test(test_1_gb, blargg_print_cb);
        do_print = true;
        jit_assert(regs.a  == 0x0);
        #endif
    }
    return 0;
}