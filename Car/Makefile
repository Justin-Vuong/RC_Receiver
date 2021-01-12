BUILD_DIR = build/
LIB_DIR = ../lib/
MCU = atmega328p
FORMAT = ihex
CLK_SPEED = 16000000UL
TARGET = main
BUILD_TARGET = $(BUILD_DIR)$(TARGET)
SRC = $(TARGET).c serialPort.c spi.c nRFL01.c

OBJ = $(SRC:%.c=build/%.o)

.DEFAULT_GOAL := $(BUILD_TARGET).hex

CC = avr-gcc
AVR-GCC_FLAGS = -Os -DF_CPU=$(CLK_SPEED) -mmcu=$(MCU)
AVR_GCC_EXEC_FLAGS = -mmcu=$(MCU) $(OBJ) 
AVR_GCC_EXEC_TARGET = -o $(BUILD_TARGET)

OBJCOPY = avr-objcopy
REMOVE = rm -f

AVRDUDE = avrdude
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB1
AVRDUDE_WRITE_FLASH = -U flash:w:$(BUILD_TARGET).hex:i
AVRDUDE_FLAGS = -F -V -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -b 57600

$(BUILD_DIR)%.o : $(LIB_DIR)%.c
	$(CC) $(AVR-GCC_FLAGS) -c $< -o $@

$(BUILD_DIR)%.o : %.c
	$(CC) $(AVR-GCC_FLAGS) -I ../lib -c $< -o $@

$(BUILD_TARGET) : $(OBJ)
	$(CC) $(AVR_GCC_EXEC_FLAGS) $(AVR_GCC_EXEC_TARGET)

$(BUILD_TARGET).hex : $(BUILD_TARGET)
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $(BUILD_TARGET) $(BUILD_TARGET).hex

all: $(BUILD_TARGET).hex

clean:
	$(REMOVE) $(BUILD_DIR)*

load: $(BUILD_TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) 
