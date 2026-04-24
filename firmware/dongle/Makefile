FX2LIBDIR=./fx2lib
BASENAME = openrtd
SOURCES = main.c i2c_bitbang.c
A51_SOURCES = dscr.a51 i2c_fast.a51
SDCCFLAGS = --opt-code-speed --peep-asm --allow-unsafe-read --peep-return

include $(FX2LIBDIR)/lib/fx2.mk

flash: $(BUILDDIR)/$(BASENAME).ihx
	sudo cycfx2prog prg:$< run
