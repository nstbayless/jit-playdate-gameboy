#include "pd_api.h"
#include "jit.h"
#include <stdint.h>

static volatile int b = 0;
static PlaydateAPI* playdate;
static uint8_t wram[0x2000];
static uint8_t hram[0x7F];

static jit_regfile_t regs;
static unsigned halt;

static const uint8_t gbrom_nop[] = {
	0x10, 	// stop
};

static const uint8_t gbrom_ld[] = {
	0x3E, 0x69,  		// ld a, $69
	0x06, 0x3A,  		// ld b, $3A
	0x11, 0x34, 0x12,   // ld de, #$1234
	0x10, 				// stop
};

static const uint8_t gbrom_transfer[] = {
	0x16, 0x20,  		// ld d, $20
	0x5A,               // ld e, d
	0x42,               // ld b, d
	0x4B,               // ld c, e
	0x61,               // ld h, c
	0x68,               // ld l, b
	0x1E, 0x33,  		// ld e, $33
	0x53,               // ld d, e
	0x10, 				// stop
};

static const uint8_t gbrom_memaccess[] = {
	0x3E, 0x30,  		// ld a, $30
	0xEA, 0x00, 0xC0,   // ld ($C000), a
	0x3E, 0x10,  		// ld a, $10
	0xFA, 0x00, 0xC0,   // ld a, ($C000)
	0x10, 				// stop
};

static void test_stop(void)
{
	halt = 1;
	playdate->system->logToConsole("STOP\n");
}

static void test_halt(void)
{
	playdate->system->logToConsole("HALT\n");
}

static void test_illegal(void)
{
	playdate->system->logToConsole("ILLEGAL\n");
}

static uint8_t test_read(uint16_t addr)
{
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

static void spin(void)
{
    for (size_t i = 0; i < 0x40000; ++i) asm("nop");
}

void do_test(const uint8_t* rom)
{
	jit_opts opts = {
		.rom = rom,
		.wram = wram,
		.hram = hram,
		.stop = test_stop,
		.halt = test_halt,
		.illegal = test_illegal,
		.read = test_read,
		.write = test_write,
		.regs = &regs,
		.is_gb_color = 0,
		.playdate = playdate
	};
	regs.pc = 0;
	halt = 0;
	
	playdate->system->logToConsole("Initialize jit.\n");
	spin();
	jit_init(opts);
	
	#if TARGET_PLAYDATE
	while (!halt)
	{
		playdate->system->logToConsole("Getting jit at %x.\n", regs.pc);
		spin();
		jit_fn fn = jit_get(regs.pc, 0);
		playdate->system->logToConsole("Done.\n");
		spin();
		if (fn)
		{
			playdate->system->logToConsole("invoking %x.\n", (void*)fn);
			spin();
			playdate->system->logToConsole("(invoking.)\n");
			spin();
			fn();
			//((jit_fn)(func))();
			playdate->system->logToConsole("Done.\n");
			spin();
			playdate->system->logToConsole("(Done.)\n");
			spin();
			break;
		}
		else
		{
			playdate->system->logToConsole("jit failed.\n");
			break;
		}
	}
	#else
	(void)jit_get(regs.pc, 0);
	#endif
	playdate->system->logToConsole("Test complete.");
	spin();
	jit_cleanup();
}

int update(void* A)
{
	jit_memfix();
	return 0;
} 

// This just serves to prove that a dummy function can be
// executed from machine code directly.
// try invoking this as a function with signature void(void).
// It can also be invoked as an int(int), in which case it returns
// the input.

typedef void(*fn_t)(void);
#ifdef TARGET_PLAYDATE
const uint16_t _nop_data[] = {
	0x4770 // bx lr
};

fn_t _nop = (fn_t)&_nop_data;
#endif

int eventHandler
(PlaydateAPI* pd, PDSystemEvent event, uint32_t arg)
{
	playdate = pd;
	playdate->system->setUpdateCallback(update, NULL);
	
	return 0;
	
	#ifdef TARGET_PLAYDATE
	jit_invalidate_cache();
	_nop();
	#endif
	//playdate->system->logToConsole("static nop: %d", _nop_ii(4));
	return 0;
	
	if (event == kEventInit)
	{
		playdate->system->setAutoLockDisabled(0);
		playdate->system->logToConsole("Nop:");
		do_test(gbrom_nop);
		
		return 0;
		
		playdate->system->logToConsole("Ld:");
		do_test(gbrom_ld);
		
		#ifdef TARGET_PLAYDATE
		playdate->system->logToConsole("register af: %4X", regs.af);
		playdate->system->logToConsole("register bc: %4X", regs.bc);
		playdate->system->logToConsole("register de: %4X", regs.de);
		playdate->system->logToConsole("register hl: %4X", regs.hl);
		jit_assert(regs.a == 0x69);
		jit_assert(regs.bc == 0x003A);
		jit_assert(regs.de == 0x1234);
		#endif
		
		playdate->system->logToConsole("Transfer:");
		do_test(gbrom_transfer);
		#ifdef TARGET_PLAYDATE
		playdate->system->logToConsole("register af: %4X", regs.af);
		playdate->system->logToConsole("register bc: %4X", regs.bc);
		playdate->system->logToConsole("register de: %4X", regs.de);
		playdate->system->logToConsole("register hl: %4X", regs.hl);
		
		jit_assert(regs.a == 0x20);
		jit_assert(regs.bc == 0x2020);
		jit_assert(regs.de == 0x3333);
		jit_assert(regs.hl == 0x2020);
		#endif
		
		return 0;
		
		playdate->system->logToConsole("Mem Access:");
		do_test(gbrom_memaccess);
		#ifdef TARGET_PLAYDATE
		playdate->system->logToConsole("register af: %4X", regs.af);
		playdate->system->logToConsole("register bc: %4X", regs.bc);
		playdate->system->logToConsole("register de: %4X", regs.de);
		playdate->system->logToConsole("register hl: %4X", regs.hl);
		
		jit_assert(regs.af == 0x30);
		jit_assert(wram[0] == 0x30);
		jit_assert(1==2);
		#endif
	}
	return 0;
}