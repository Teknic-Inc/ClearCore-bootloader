include helper_functions.mk

BOARD        := ClearCore
CHIP_FAMILY  := same53
CHIP_VARIANT := SAME53N19A

CC           := arm-none-eabi-gcc
COMMON_FLAGS := -mthumb -mcpu=cortex-m4 -O2 -g3 -D$(call to-upper,$(CHIP_FAMILY))=1
WFLAGS       := \
-Werror -Wall -Wstrict-prototypes \
-Werror-implicit-function-declaration -Wpointer-arith -std=gnu11 \
-ffunction-sections -fdata-sections -Wchar-subscripts -Wcomment -Wformat=2 \
-Wimplicit-int -Wmain -Wparentheses -Wsequence-point -Wreturn-type -Wswitch \
-Wtrigraphs -Wunused -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wno-undef \
-Wbad-function-cast -Wwrite-strings -Waggregate-return \
-Wformat -Wmissing-format-attribute \
-Wno-deprecated-declarations -Wpacked -Wredundant-decls -Wnested-externs \
-Wlong-long -Wunreachable-code -Wcast-align \
-Wno-missing-braces -Wno-overflow -Wno-shadow -Wno-attributes -Wno-packed -Wno-pointer-sign

CFLAGS := \
  $(COMMON_FLAGS) \
  -x c -c -pipe -nostdlib \
  --param max-inline-insns-single=500 \
  -fno-strict-aliasing -fdata-sections -ffunction-sections \
  -D__$(CHIP_VARIANT)__ \
  -DUSE_HID=1 \
  -DUSE_LOGS=1 \
  $(WFLAGS)

UF2_VERSION_BASE := $(shell git describe --dirty --always --tags)

LINKER_SCRIPT      := scripts/$(call to-lower,$(CHIP_VARIANT)).ld
BOOTLOADER_SIZE    := 16384
SELF_LINKER_SCRIPT := scripts/$(call to-lower,$(CHIP_VARIANT))_self.ld

LDFLAGS := $(COMMON_FLAGS) \
  -Wall -Wl,--cref -Wl,--check-sections -Wl,--gc-sections \
  -Wl,--unresolved-symbols=report-all -Wl,--warn-common \
  -Wl,--warn-section-align \
  -save-temps -nostartfiles \
  --specs=nano.specs --specs=nosys.specs

BUILD_PATH := build/$(BOARD)
INCLUDES   := \
  -I. \
  -I./inc \
  -I./boards/$(BOARD) \
  -Ilib/cmsis/CMSIS/Include \
  -Ilib/usb_msc \
  -Ilib/$(CHIP_FAMILY)/include \
  -I$(BUILD_PATH)


COMMON_SRC := \
  src/flash_$(CHIP_FAMILY).c \
  src/init_$(CHIP_FAMILY).c \
  src/startup_$(CHIP_FAMILY).c \
  src/usart_sam_ba.c \
  src/screen.c \
  src/utils.c

SOURCES := $(COMMON_SRC) \
  src/cdc_enumerate.c \
  src/fat.c \
  src/main.c \
  src/msc.c \
  src/sam_ba_monitor.c \
  src/uart_driver.c \
  src/hid.c

OBJECTS := $(patsubst src/%.c,$(BUILD_PATH)/%.o,$(SOURCES))

NAME                 := bootloader-$(BOARD)-$(UF2_VERSION_BASE)
EXECUTABLE           := $(BUILD_PATH)/$(NAME).bin

SUBMODULES := lib/uf2/README.md

all: $(SUBMODULES) $(EXECUTABLE)

-include Makefile.user

$(BUILD_PATH)/flash.jlink: $(BUILD_PATH)/$(NAME).bin
	echo " \n\
r \n\
h \n\
loadbin \"$(BUILD_PATH)/$(NAME).bin\", 0x0 \n\
verifybin \"$(BUILD_PATH)/$(NAME).bin\", 0x0 \n\
r \n\
qc \n\
" > $(BUILD_PATH)/flash.jlink

jlink-flash: $(BUILD_PATH)/$(NAME).bin $(BUILD_PATH)/flash.jlink
	JLinkExe -if swd -device AT$(CHIP_VARIANT) -speed 4000 -CommanderScript $(BUILD_PATH)/flash.jlink

$(EXECUTABLE): $(OBJECTS)
	$(CC) -L$(BUILD_PATH) $(LDFLAGS) \
	  -T$(LINKER_SCRIPT) \
	  -Wl,-Map,$(BUILD_PATH)/$(NAME).map -o $(BUILD_PATH)/$(NAME).elf $(OBJECTS)
	arm-none-eabi-objcopy -O binary $(BUILD_PATH)/$(NAME).elf $@
	@echo
	-@arm-none-eabi-size $(BUILD_PATH)/$(NAME).elf | awk '{ s=$$1+$$2; print } END { print ""; print "Space left: " ($(BOOTLOADER_SIZE)-s) }'
	@echo

$(BUILD_PATH)/uf2_version.h: Makefile | $(BUILD_PATH)
	echo "#define UF2_VERSION_BASE \"$(UF2_VERSION_BASE)\""> $@

$(BUILD_PATH)/%.o: src/%.c $(wildcard inc/*.h boards/*/*.h) $(BUILD_PATH)/uf2_version.h
	echo "$<"
	$(CC) $(CFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@

$(BUILD_PATH)/%.o: $(BUILD_PATH)/%.c
	$(CC) $(CFLAGS) $(BLD_EXTA_FLAGS) $(INCLUDES) $< -o $@

$(OBJECTS): | $(BUILD_PATH)

$(BUILD_PATH):
	mkdir -p $@

clean:
	rm -rf build

$(SUBMODULES):
	git submodule update --init --recursive
