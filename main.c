#include "pd_api.h"
#include <stdint.h>

static volatile int b = 0;

void  __attribute__ ((noinline))
functest()
{
	if (b > 0)
	{
		functest();
	}
}

uint16_t func[] = {
	0x4770,
};

typedef void(*fn)(void);

int update(void* A)
{
	return 0;
}

int eventHandler
(PlaydateAPI* playdate, PDSystemEvent event, uint32_t arg)
{
	playdate->system->logToConsole("hello %d\n", event);
	playdate->system->setUpdateCallback(update, NULL);
	
	#ifdef TARGET_PLAYDATE
	if (event == kEventInit)
	{
		playdate->system->logToConsole("about to call\n");
		((fn)(func))();
		playdate->system->logToConsole("call succeeded\n");
	}
	#endif
	return 0;
}