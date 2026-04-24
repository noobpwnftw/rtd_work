CC      = sdcc
MAKEBIN = makebin

TARGET  = stub
SRC     = $(TARGET).c

CFLAGS  = -mmcs51 --no-xinit-opt

.PHONY: all clean

all: $(TARGET).bin

$(TARGET).ihx: $(SRC)
	$(CC) $(CFLAGS) -o $(TARGET).ihx $(SRC)

$(TARGET).bin: $(TARGET).ihx
	$(MAKEBIN) -s 65536 $(TARGET).ihx $(TARGET).bin

clean:
	rm -f $(TARGET).ihx $(TARGET).bin $(TARGET).asm $(TARGET).lst $(TARGET).sym $(TARGET).map $(TARGET).mem $(TARGET).rst $(TARGET).rel $(TARGET).lk
