# SPDX-License-Identifier: (GPL-2.0+ OR MIT)

.SUFFIXES:
.PHONY: all debug release clean

# Set basic variables
PROJ := slboot
DEVICE ?= EFM32JG1B200F128GM48
CROSS_COMPILE ?= /opt/arm/gcc-arm-none-eabi/7.3-2018-q2/bin/arm-none-eabi-

# Include a config.mk if it exists
-include config.mk

# TODO: remove this!
uniq = $(strip $(if $1,$(firstword $1) \
       $(call uniq,$(filter-out $(firstword $1),$1))))

ifeq ($(VERBOSE),)
	ECHO = @
endif

# Some basic tools
MK	:= mkdir -p
MV	:= mv
CP	:= cp -ar
RM	:= rm -rf
NULL	:= /dev/null

# Some basic directories
OUT_DIR = .zout
OBJ_DIR = $(OUT_DIR)/build
EXE_DIR = $(OUT_DIR)

$(shell $(MK) $(OBJ_DIR) >/dev/null 2>&1)
$(shell $(MK) $(EXE_DIR) >/dev/null 2>&1)

ifeq (clean,$(findstring clean, $(MAKECMDGOALS)))
  ifneq ($(filter $(MAKECMDGOALS),all debug release),)
    $(shell $(RM) $(OBJ_DIR) >/dev/null 2>&1)
    $(shell $(RM) $(EXE_DIR) >/dev/null 2>&1)
  endif
endif

CC	:= $(CROSS_COMPILE)gcc
CPP	:= $(CROSS_COMPILE)g++
AR	:= $(CC)
LD	:= $(CC)

OBJSIZE	:= $(CROSS_COMPILE)size
OBJCOPY	:= $(CROSS_COMPILE)objcopy
OBJDUMP	:= $(CROSS_COMPILE)objdump

DEPFLAGS = -MMD -MP -MF$(@:.o=.d) -MT$(@)

override ARFLAGS = \
	-D__STACK_SIZE=0x400 -Wstack-usage=1024 -fstack-usage \
	-x assembler-with-cpp -Wall -Wextra -mcpu=cortex-m3 \
	-mthumb -DDEBUG_EFM -D$(DEVICE) $(DEPFLAGS)

override CFLAGS = \
	-Wall -Wextra -mcpu=cortex-m3 -mthumb \
	-mfix-cortex-m3-ldrd -ffunction-sections \
	-fdata-sections -fomit-frame-pointer -std=c99 -DDEBUG_EFM  \
	-fsigned-char -fmessage-length=0 \
	-Wstack-usage=1024 -fstack-usage \
	-Wa,-ahld=$(OBJ_DIR)/$(@F:.o=.lst) \
	-D$(DEVICE) $(DEPFLAGS)

override LDFLAGS = \
	-Xlinker -Map=$(EXE_DIR)/$(PROJ).map -mcpu=cortex-m3 \
	-mthumb -Tarch/efm32jg1b/Source/GCC/efm32jg1b.ld \
	--specs=nano.specs --specs=nosys.specs  \
	-Wl,--gc-sections 

LIBS = -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group

INC_DIRS := \
	-Isrc/inc \
	-Isrc/cfg \
	-Isrc/cfg/emdrv \
	-Isrc/cfg/segger \
	-Iarch/arm/Include \
	-Iarch/efm32jg1b/Include \
	-Ilibs/emlib/inc \
	-Idrivers/segger-rtt \
	-Idrivers/emdrv/common/inc \
	-Idrivers/emdrv/tempdrv/inc \

C_SRC +=  \
	src/boot.c \
	src/fatal.c \
	libs/emlib/src/em_cmu.c \
	libs/emlib/src/em_rmu.c \
	libs/emlib/src/em_emu.c \
	libs/emlib/src/em_gpio.c \
	libs/emlib/src/em_wdog.c \
	libs/emlib/src/em_system.c \
	libs/emlib/src/em_assert.c \
	drivers/emdrv/tempdrv/src/tempdrv.c \
	drivers/segger-rtt/SEGGER_RTT.c \
	drivers/segger-rtt/SEGGER_RTT_printf.c \
	arch/efm32jg1b/Source/system_efm32jg1b.c

S_SRC +=  \
	arch/efm32jg1b/Source/GCC/startup_efm32jg1b.S

C_FILES = $(notdir $(C_SRC))
S_FILES = $(notdir $(S_SRC))

C_PATHS = $(call uniq, $(dir $(C_SRC)))
S_PATHS = $(call uniq, $(dir $(S_SRC)))

C_OBJS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.o))
S_OBJS = $(addprefix $(OBJ_DIR)/, $(S_FILES:.S=.o))
C_DEPS = $(addprefix $(OBJ_DIR)/, $(C_FILES:.c=.d))
S_DEPS = $(addprefix $(OBJ_DIR)/, $(S_FILES:.S=.d))
OBJS = $(C_OBJS) $(S_OBJS)

vpath %.c $(C_PATHS)
vpath %.S $(S_PATHS)

all:	debug

debug:	CFLAGS += -DDEBUG -O0 -g
debug:	$(EXE_DIR)/$(PROJ).bin $(EXE_DIR)/$(PROJ).hex

release:	CFLAGS += -O0 -g
release:	$(EXE_DIR)/$(PROJ).bin

clean:
ifeq ($(filter $(MAKECMDGOALS),all debug release),)
	$(ECHO)$(RM) -v $(OUT_DIR)
endif

print:
	$(foreach v, $(.VARIABLES), $(info $(v) = $($(v))))

ifneq (clean,$(findstring clean, $(MAKECMDGOALS)))
-include $(C_DEPS)
-include $(S_DEPS)
endif

$(OBJ_DIR)/%.o: %.S
	@echo "AR $(shell basename $<)"
	$(ECHO)$(AR) $(ARFLAGS) $(INC_DIRS) -c -o $@ $<
	$(ECHO)$(AR) $(ARFLAGS) $(INC_DIRS) -E $< -o $(OBJ_DIR)/$(@F:.o=.i)

$(OBJ_DIR)/%.o: %.c
	@echo "CC $(shell basename $<)"
	$(ECHO)$(CC) $(CFLAGS) $(INC_DIRS) -c -o $@ $<
	$(ECHO)$(CC) $(CFLAGS) $(INC_DIRS) -E $< -o $(OBJ_DIR)/$(@F:.o=.i)

$(EXE_DIR)/$(PROJ): $(OBJS)
	@echo "LD $(shell basename $@)"
	$(ECHO)$(LD) $(LDFLAGS) $(OBJS) $(LIBS) -o $(EXE_DIR)/$(PROJ)

$(EXE_DIR)/$(PROJ).elf: $(EXE_DIR)/$(PROJ)
	@echo "CP $(shell basename $@)"
	$(ECHO)$(CP) $< $@
	$(ECHO)$(OBJDUMP) -h -S -C $@ >$(EXE_DIR)/$(PROJ).lst
	$(ECHO)$(OBJSIZE) --format=berkeley $@ >$(EXE_DIR)/$(PROJ).size

$(EXE_DIR)/$(PROJ).bin: $(EXE_DIR)/$(PROJ).elf
	@echo "BIN $(shell basename $@)"
	$(ECHO)$(OBJCOPY) -O binary $< $(EXE_DIR)/$(PROJ).bin

$(EXE_DIR)/$(PROJ).hex: $(EXE_DIR)/$(PROJ).elf
	@echo "HEX $(shell basename $@)"
	$(ECHO)$(OBJCOPY) -O ihex $< $(EXE_DIR)/$(PROJ).hex
