BUILD_DIR = build/

MCU = atmega328p
FORMAT = ihex
TARGET = main
BUILD_TARGET = $(BUILD_DIR)$(TARGET)
SRC = $(TARGET).c 
OBJ = $(BUILD_DIR)$(SRC:.c=.o)

.DEFAULT_GOAL := $(BUILD_TARGET).hex

AVRDUDE = avrdude
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0

AVRDUDE_WRITE_FLASH = -U flash:w:$(BUILD_TARGET).hex:i

AVRDUDE_FLAGS = -F -V -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -b 57600

CC = avr-gcc
AVR-GCC_CLK_SPEED = 16000000UL


OBJCOPY = avr-objcopy
REMOVE = rm -f

AVR-GCC_FLAGS = -Os -DF_CPU=$(AVR-GCC_CLK_SPEED) -mmcu=$(MCU)
AVR_GCC_EXEC_FLAGS = -mmcu=$(MCU) $(BUILD_TARGET).o 
AVR_GCC_EXEC_TARGET = -o $(BUILD_TARGET)

$(BUILD_DIR)%.o : %.c
	$(CC) $(AVR-GCC_FLAGS) -c $< -o $@

$(BUILD_TARGET) : $(BUILD_TARGET).o
	$(CC) $(AVR_GCC_EXEC_FLAGS) $(AVR_GCC_EXEC_TARGET)
	echo $(BUILD_TARGET)

$(BUILD_TARGET).hex : $(BUILD_TARGET)
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $(BUILD_TARGET) $(BUILD_TARGET).hex

all: $(BUILD_TARGET).hex

clean:
	$(REMOVE) $(BUILD_DIR)*

load: $(BUILD_TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) 
