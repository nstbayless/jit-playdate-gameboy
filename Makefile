ifeq ($(HEAP_SIZE),)
HEAP_SIZE      = 8388208
endif
ifeq ($(STACK_SIZE),)
STACK_SIZE     = 61800
endif

ifeq ($(PRODUCT),)
PRODUCT = JITTest.pdx
endif

SELF_DIR := $(dir $(lastword $(MAKEFILE_LIST)))


SDK = ${PLAYDATE_SDK_PATH}
ifeq ($(SDK),)
SDK = $(shell egrep '^\s*SDKRoot' ~/.Playdate/config | head -n 1 | cut -c9-)
endif

ifeq ($(SDK),)
$(error SDK path not found; set ENV value PLAYDATE_SDK_PATH)
endif

VPATH +=

SRCC = main.c jit.c armd.c sm38d.c

# List C source files here
SRC += $(SRCC) pdnewlib.c

# List all user directories here
UINCDIR += $(VPATH)

# List all user C define here, like -D_DEBUG=1
UDEFS += -DJIT_DEBUG

# Define ASM defines here
UADEFS +=

# List the user directory to look for the libraries here
ULIBDIR +=

# List all user libraries here
ULIBS +=

CLANGFLAGS +=

include $(SDK)/C_API/buildsupport/common.mk

jt_arm: $(SRCC)
	arm-none-eabi-gcc $(SRCC) shnewlib.c $(UDEFS) -mthumb -mcpu=cortex-m3 -march=armv7-m -T qemu.ld -Wl,-Map=output.map -DTARGET_QEMU -g  -o jt_arm
