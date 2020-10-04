MCU = atmega328p
FORMAT = ihex
TARGET = HelloWorld
SRC = $(TARGET).c 
OBJ = $(SRC:.c=.o)

AVRDUDE = avrdude
AVRDUDE_PROGRAMMER = arduino
AVRDUDE_PORT = /dev/ttyUSB0

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex:i

AVRDUDE_FLAGS = -F -V -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER) -b 57600

CC = avr-gcc
AVR-GCC_CLK_SPEED = 16000000UL


OBJCOPY = avr-objcopy
REMOVE = rm -f

AVR-GCC_FLAGS = -Os -DF_CPU=$(AVR-GCC_CLK_SPEED) -mmcu=$(MCU)
AVR_GCC_EXEC_FLAGS = -mmcu=$(MCU) $(TARGET).o 
AVR_GCC_EXEC_TARGET = -o $(TARGET)

%.o : %.c
	$(CC) $(AVR-GCC_FLAGS) -c $< -o $@

$(TARGET): $(TARGET).o
	$(CC) $(AVR_GCC_EXEC_FLAGS) $(AVR_GCC_EXEC_TARGET)

$(TARGET).hex: $(TARGET)
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $(TARGET) $(TARGET).hex

clean:
	$(REMOVE) $(TARGET) $(TARGET).hex $(OBJ)

load: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) 
