# ----------------------------------------------------------------------------
#         ATMEL Microcontroller Software Support 
# ----------------------------------------------------------------------------
# Copyright (c) 2008, Atmel Corporation
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice,
# this list of conditions and the disclaimer below.
#
# Atmel's name may not be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

# 	Makefile for compiling the USB CDC serial project

#-------------------------------------------------------------------------------
#		User-modifiable options
#-------------------------------------------------------------------------------

# Chip & board used for compilation
# (can be overriden by adding CHIP=chip and BOARD=board to the command-line)
BOARD ?= tfrog-rev5

chip = case "$(BOARD)" in \
	tfrog-rev5) echo at91sam7se256;;\
	tfrog-rev4) echo at91sam7se512;;\
	*) echo at91sam7se512;;\
esac
CHIP := $(shell $(chip))


# Trace level used for compilation
# (can be overriden by adding TRACE_LEVEL=#number to the command-line)
# TRACE_LEVEL_DEBUG      5
# TRACE_LEVEL_INFO       4
# TRACE_LEVEL_WARNING    3
# TRACE_LEVEL_ERROR      2
# TRACE_LEVEL_FATAL      1
# TRACE_LEVEL_NO_TRACE   0
TRACE_LEVEL = 2

# Optimization level, put in comment for debugging
CPUSPEC = -mcpu=arm7tdmi -marm
OPTIMIZATION = -O3 -fno-aggressive-loop-optimizations
# -fomit-frame-pointer -funroll-loops

# AT91 library directory
AT91LIB = ../at91lib

# Output file basename
OUTPUT = tfrog-motordriver-$(BOARD)-$(CHIP)

# Compile for all memories available on the board (this sets $(MEMORIES))
include $(AT91LIB)/boards/$(BOARD)/board.mak

# Output directories
BIN = bin
OBJ = obj

#-------------------------------------------------------------------------------
#		Tools
#-------------------------------------------------------------------------------

# Tool suffix when cross-compiling
COMPILER_PREFIX = /usr/local/arm-none-eabi/
CROSS_COMPILE = arm-none-eabi-
#arm-elf-

# Compilation tools
CC = $(CROSS_COMPILE)gcc
SIZE = $(CROSS_COMPILE)size
OBJCOPY = $(CROSS_COMPILE)objcopy

# Flags
INCLUDES = -I$(AT91LIB)/boards/$(BOARD) -I$(AT91LIB)/peripherals
INCLUDES += -I$(AT91LIB)/components -I$(AT91LIB)/usb/device -I$(AT91LIB)
INCLUDES += -I$(AT91LIB)/memories

FIRMINFO ?= 

CFLAGS = -Wall -mlong-calls $(FIRMINFO) #-ffunction-sections
CFLAGS += $(CPUSPEC) $(OPTIMIZATION) $(INCLUDES) -D$(CHIP) -DTRACE_LEVEL=$(TRACE_LEVEL)
CFLAGS2 = -Wall -mlong-calls $(FIRMINFO) #-ffunction-sections
CFLAGS2 += $(CPUSPEC) -O0 $(INCLUDES) -D$(CHIP) -DTRACE_LEVEL=$(TRACE_LEVEL)
ASFLAGS = $(CPUSPEC) -O0 $(INCLUDES) -D$(CHIP) -D__ASSEMBLY__
LDFLAGS = $(CPUSPEC) -O0 -nostartfiles -Wl,--gc-sections -L$(COMPILER_PREFIX)/lib

#-------------------------------------------------------------------------------
#		Files
#-------------------------------------------------------------------------------

# Directories where source files can be found
USB = $(AT91LIB)/usb
UTILITY = $(AT91LIB)/utility
PERIPH = $(AT91LIB)/peripherals
MEM = $(AT91LIB)/memories
BOARDS = $(AT91LIB)/boards

VPATH += $(USB)/device/cdc-serial $(USB)/device/core $(USB)/common/core
VPATH += $(USB)/common/cdc
VPATH += $(UTILITY)
VPATH += $(PERIPH)/dbgu $(PERIPH)/aic $(PERIPH)/usart $(PERIPH)/pio $(PERIPH)/pmc $(PERIPH)/efc $(PERIPH)/eefc
VPATH += $(MEM)/flash
VPATH += $(BOARDS)/$(BOARD) $(BOARDS)/$(BOARD)/$(CHIP)

# Objects built from C source files
C_OBJECTS = main.o
C_OBJECTS += CDCDSerialDriver.o CDCDSerialDriverDescriptors.o
C_OBJECTS += CDCSetControlLineStateRequest.o CDCLineCoding.o
C_OBJECTS += USBD_OTGHS.o USBD_UDP.o USBD_UDPHS.o USBDDriver.o
C_OBJECTS += USBDCallbacks_Initialized.o
C_OBJECTS += USBDCallbacks_Reset.o
#C_OBJECTS += USBDCallbacks_Resumed.o
#C_OBJECTS += USBDCallbacks_Suspended.o
C_OBJECTS += USBDDriverCb_CfgChanged.o
C_OBJECTS += USBDDriverCb_IfSettingChanged.o
C_OBJECTS += USBSetAddressRequest.o USBGenericDescriptor.o USBInterfaceRequest.o
C_OBJECTS += USBGenericRequest.o USBGetDescriptorRequest.o 
C_OBJECTS += USBSetConfigurationRequest.o USBFeatureRequest.o
C_OBJECTS += USBEndpointDescriptor.o USBConfigurationDescriptor.o
C_OBJECTS += led.o string.o stdio.o
C_OBJECTS += flashd_efc.o flashd_eefc.o
C_OBJECTS += aic.o dbgu.o usart.o pio.o pio_it.o pmc.o efc.o eefc.o 
C_OBJECTS += power.o math.o filter.o
C_OBJECTS += eeprom.o adc.o io.o
C_OBJECTS += controlVelocity.o controlPWM.o communication.o
C_OBJECTS += fixpawd.o fixpawd_math.o

# Low level object files (don't optimize them)
C_OBJECTS2 = board_lowlevel.o board_memories.o

# Objects built from Assembly source files
ASM_OBJECTS = board_cstartup.o
#ASM_OBJECTS += cp15_asm.o

# Append OBJ and BIN directories to output filename
OUTPUT := $(BIN)/$(OUTPUT)

#-------------------------------------------------------------------------------
#		Rules
#-------------------------------------------------------------------------------

all: $(BIN) $(OBJ) $(MEMORIES)

$(BIN) $(OBJ):
	mkdir $@

define RULES
C_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS))
C_OBJECTS2_$(1) = $(addprefix $(OBJ)/$(1)_, $(C_OBJECTS2))
ASM_OBJECTS_$(1) = $(addprefix $(OBJ)/$(1)_, $(ASM_OBJECTS))

$(1): $$(ASM_OBJECTS_$(1)) $$(C_OBJECTS_$(1)) $$(C_OBJECTS2_$(1))
	$(CC) $(LDFLAGS) -T"$(AT91LIB)/boards/$(BOARD)/$(CHIP)/$$@.lds" -o $(OUTPUT)-$$@.elf $$^
	$(OBJCOPY) -O binary $(OUTPUT)-$$@.elf $(OUTPUT)-$$@.bin
	$(SIZE) $$^ $(OUTPUT)-$$@.elf

$$(C_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS) -D$(1) -c -o $$@ $$<

$$(C_OBJECTS2_$(1)): $(OBJ)/$(1)_%.o: %.c Makefile $(OBJ) $(BIN)
	$(CC) $(CFLAGS2) -D$(1) -c -o $$@ $$<

$$(ASM_OBJECTS_$(1)): $(OBJ)/$(1)_%.o: %.S Makefile $(OBJ) $(BIN)
	$(CC) $(ASFLAGS) -D$(1) -c -o $$@ $$<

debug_$(1): $(1)
	perl ../resources/gdb/debug.pl $(OUTPUT)-$(1).elf

endef

$(foreach MEMORY, $(MEMORIES), $(eval $(call RULES,$(MEMORY))))

clean:
	-rm -f $(OBJ)/*.o $(BIN)/*.elf
#$(BIN)/*.bin

