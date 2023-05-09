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

# List C source files here
SRC += main.c jit.c armd.c sm38d.c pdnewlib.c

# List all user directories here
UINCDIR += $(VPATH) ./STM32CubeF7/Drivers/CMSIS/Include ./STM32CubeF7/Drivers/CMSIS/Device/ST/STM32F7xx/Include

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