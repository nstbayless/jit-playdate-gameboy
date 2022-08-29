#include "pd_api.h"
#include "jit.h"
#include <stdint.h>

static volatile int b = 0;
static PlaydateAPI* playdate;
static uint8_t wram[0x800];
static uint8_t hram[0x7F];

static jit_regfile regs;
static unsigned halt;

static const uint8_t gbrom_nop[] = {
	0x10, 	// stop
};

static const uint8_t gbrom_lda[] = {
	0x3E, 0x69  // ld a, $69
	0x10, 		// stop
};

void test_stop(void)
{
	halt = 1;
	playdate->system->logToConsole("STOP\n");
}

void test_halt(void)
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


int eventHandler
(PlaydateAPI* pd, PDSystemEvent event, uint32_t arg)
{
	playdate = pd;
	playdate->system->logToConsole("hello %d\n", event);
	playdate->system->setUpdateCallback(update, NULL);
	
	if (event == kEventInit)
	{
		do_test(gbrom_nop);
		
		do_test(gbrom_lda);
		#ifdef TARGET_PLAYDATE
		playdate->system->logToConsole("call succeeded\n");
		#endif
	}
	return 0;
}