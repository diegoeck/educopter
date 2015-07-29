TARGET=codigo
SRCS=$(TARGET).c mi2c.c uart.c 

PROCESSOR=atmega328p
CLOCK=16000000
SERIAL=57600
COM=/dev/tty.NAMIMOTE001-DevB
COM=/dev/tty.usbserial-A7035F2Y

FLAGS=-Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(PROCESSOR)
INCLUDE=
LIBS=-lm

CC=/usr/local/CrossPack-AVR/bin/avr-gcc
OC=/usr/local/CrossPack-AVR/bin/avr-objcopy
DUDE=/usr/local/CrossPack-AVR/bin/avrdude

CFLAGS=$(FLAGS) $(INCLUDE)
LDFLAGS=$(LIBS)

all: $(TARGET).hex

$(TARGET).hex:$(SRCS:.c=.o)
	$(CC) $(CFLAGS) -o $(TARGET).elf $^ $(LDFLAGS)
	$(OC) -O ihex $(TARGET).elf $(TARGET).hex

%.o:%.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash:all
	$(DUDE) -c arduino -p $(PROCESSOR) -b$(SERIAL) -P $(COM) -U flash:w:$(TARGET).hex

clear:
	rm -f *.o *.a *.elf

distclear:clear
	rm $(TARGET).hex

