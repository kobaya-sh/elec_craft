MMCU	=	atmega8
ABBREV	=	atmega8
#MMCU	=	attiny13a
#ABBREV	=	t13
F_CPU	=	1000000
WRITER	=	usbasp

PROGRAM	=	timer

BINDIR	=	$(HOME)/avr-develop-kit/avr-gcc/bin/

CC	=	$(BINDIR)/avr-gcc
OBJCOPY	=	$(BINDIR)/avr-objcopy
AVRDUDE	=	$(BINDIR)/avrdude

CFLAGS	=	-O2 -mmcu=$(MMCU) -DF_CPU=$(F_CPU)
LFLAGS	=

OBJS	=	$(PROGRAM).o

FUSE	=	-U lfuse:w:0xef:m -U hfuse:w:0xd9:m

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
	$(AVRDUDE) -c $(WRITER) -p $(ABBREV) $(FUSE) -U flash:w:$<

clean:
	/bin/rm -f $(OBJS) $(PROGRAM).x $(PROGRAM).hex

