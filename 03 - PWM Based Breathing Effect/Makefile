# Project Settings
PROJECT = firmware
TARGET  = $(PROJECT).elf
BIN     = $(PROJECT).bin
HEX     = $(PROJECT).hex

# Toolchain
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)g++
OBJCOPY = $(PREFIX)objcopy
OBJDUMP = $(PREFIX)objdump
SIZE    = $(PREFIX)size

# MCU Settings
MCU       = cortex-m4
FPU       = fpv4-sp-d16
FLOAT_ABI = hard

# --- UPDATED DIRECTORIES FOR YOUR NEW STRUCTURE ---
PLATFORM_DIR = ../platform
DRIVERS_DIR  = $(PLATFORM_DIR)/drivers
BUILD_DIR    = build

# Files
# main.cpp is in the current directory
SRC_FILES    = main.cpp 
# Shared vendor files from platform
VENDOR_SRC   = $(DRIVERS_DIR)/system_stm32f4xx.c $(DRIVERS_DIR)/segger_fault_handler.c
STARTUP_FILE = $(PLATFORM_DIR)/startup_stm32f411xe.s
LDSCRIPT     = $(PLATFORM_DIR)/STM32F411CEU6_FLASH.ld

# Object files mapping
OBJ_FILES = $(BUILD_DIR)/main.o \
            $(BUILD_DIR)/system_stm32f4xx.o \
            $(BUILD_DIR)/segger_fault_handler.o \
            $(BUILD_DIR)/startup.o

# Flags
# -I$(DRIVERS_DIR) is critical so main.cpp finds stm32f411xe.h
CFLAGS = -mcpu=$(MCU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) \
         -Og -g3 -Wall -fdata-sections -ffunction-sections \
         -DSTM32F411xE \
         -I$(DRIVERS_DIR)

LDFLAGS = -mcpu=$(MCU) -mthumb -mfpu=$(FPU) -mfloat-abi=$(FLOAT_ABI) \
          -Wl,--gc-sections -T$(LDSCRIPT) \
          -Wl,-Map=$(BUILD_DIR)/$(PROJECT).map \
          -specs=nano.specs -specs=nosys.specs

# --- Rules ---
all: $(BUILD_DIR)/$(TARGET) $(BUILD_DIR)/$(BIN) $(BUILD_DIR)/$(HEX)

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Compile local main.cpp
$(BUILD_DIR)/main.o: main.cpp | $(BUILD_DIR)
	@echo "Compiling $<..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile Vendor C files from platform
$(BUILD_DIR)/%.o: $(DRIVERS_DIR)/%.c | $(BUILD_DIR)
	@echo "Compiling Vendor C file $<..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile startup assembly from platform
$(BUILD_DIR)/startup.o: $(STARTUP_FILE) | $(BUILD_DIR)
	@echo "Compiling startup file..."
	@$(CC) $(CFLAGS) -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET): $(OBJ_FILES)
	@echo "Linking..."
	@$(CC) $(OBJ_FILES) $(LDFLAGS) -o $@
	@$(SIZE) $@

# Create binary
$(BUILD_DIR)/$(BIN): $(BUILD_DIR)/$(TARGET)
	@$(OBJCOPY) -O binary $< $@

# Create hex
$(BUILD_DIR)/$(HEX): $(BUILD_DIR)/$(TARGET)
	@$(OBJCOPY) -O ihex $< $@

# Flash with DFU-UTIL
dfu: $(BUILD_DIR)/$(BIN)
	@echo "Flashing with DFU-UTIL..."
	dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(BIN)

clean:
	@rm -rf $(BUILD_DIR)

.PHONY: all clean dfu
