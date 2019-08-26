MMCU	=	atmega8
ABBREV	=	atmega8
#MMCU	=	attiny13a
#ABBREV	=	t13
F_CPU	=	1000000
WRITER	=	usbasp

#PROGRAM	=	uart
PROGRAM	=	softuart.final
#PROGRAM	=	inline

BINDIR	=	$(HOME)/avr-develop-kit/avr-gcc/bin/

CC	=	$(BINDIR)/avr-gcc
OBJCOPY	=	$(BINDIR)/avr-objcopy
AVRDUDE	=	$(BINDIR)/avrdude

CFLAGS	=	-O2 -mmcu=$(MMCU) -DF_CPU=$(F_CPU)
LFLAGS	=

OBJS	=	$(PROGRAM).o

.PHONY: all write clean
#all: $(PROGRAM).hex
all: write

$(PROGRAM).hex: $(PROGRAM).x
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

$(PROGRAM).x: $(OBJS)
	$(CC) $(CFLAGS) $(LFLAGS) $^ -o $(PROGRAM).x

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

write: $(PROGRAM).hex
	$(AVRDUDE) -c $(WRITER) -p $(ABBREV) -U flash:w:$<

clean:
	/bin/rm -f $(OBJS) $(PROGRAM).x $(PROGRAM).hex

