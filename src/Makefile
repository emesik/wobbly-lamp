NAME=wobbly-lamp

MCU=atmega8
F_CPU=8000000UL
FUSE_HIGH=0xd9
FUSE_LOW=0x84
PROGRAMMER_ID=usbasp

CC=avr-gcc
OBJCOPY=avr-objcopy
OBJDUMP=avr-objdump
AVRDUDE=avrdude
AVRSIZE=avr-size

CFLAGS=-c -I . -mmcu=$(MCU) -Os -Wall -DF_CPU=$(F_CPU) -std=c99
LDFLAGS=-mmcu=$(MCU)

all: compile elf hex size

debug: CFLAGS += -DDEBUG -g
debug: all

compile:
	$(CC) $(CFLAGS) $(NAME).c -o $(NAME).o

elf: compile
	$(CC) $(LDFLAGS) $(NAME).o -o $(NAME).elf

hex: elf
	$(OBJCOPY) -j .text -j .data -O ihex $(NAME).elf $(NAME).hex

size: elf
	$(AVRSIZE) $(NAME).elf

write:
	$(AVRDUDE) -c $(PROGRAMMER_ID) -p $(MCU) -e \
		-U flash:w:$(NAME).hex \
		-U lfuse:w:$(FUSE_LOW):m -U hfuse:w:$(FUSE_HIGH):m

clean:
	rm -f $(NAME).o $(NAME).elf $(NAME).hex $(NAME).eep.hex
