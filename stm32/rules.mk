INCDIR += . $(EFROOT)
BUILDDIR = build

CXXOBJS = $(addprefix $(BUILDDIR)/, $(notdir $(CXXSRC:.cxx=.o)))
CPPOBJS = $(addprefix $(BUILDDIR)/, $(notdir $(CPPSRC:.cpp=.o)))
COBJS = $(addprefix $(BUILDDIR)/, $(notdir $(CSRC:.c=.o)))
OBJS = $(COBJS) $(CXXOBJS) $(CPPOBJS)

OUTFILES = $(BUILDDIR)/$(PROJECT).elf $(BUILDDIR)/$(PROJECT).bin
ifneq ($(ENABLE_OUTPUT_HEX),)
OUTFILES += $(BUILDDIR)/$(PROJECT).hex
endif

OPT += -ffunction-sections -fdata-sections -fno-common

IINCDIR   = $(patsubst %,-I%,$(INCDIR))
LLIBDIR   = $(patsubst %,-L%,$(LIBDIR))

vpath %.cxx $(dir $(CXXSRC))
vpath %.cpp $(dir $(CPPSRC))
vpath %.c $(dir $(CSRC))

MCFLAGS   = -mcpu=$(MCU)

CFLAGS    = $(MCFLAGS) $(OPT) $(CWARN) -Wa,-alms=$(BUILDDIR)/$(notdir $(<:.c=.lst)) $(DEFS)

LDFLAGS = $(MCFLAGS) -nostartfiles $(EXTRA_LDFLAGS) -T$(LDSCRIPT) -Wl,-Map=$(BUILDDIR)/$(PROJECT).map,--cref,--no-warn-mismatch $(LLIBDIR)

CFLAGS   += -mthumb -mno-thumb-interwork -DTHUMB
LDFLAGS  += -mthumb -mno-thumb-interwork

CFLAGS   += -MD -MP -MF .dep/$(@F).d
CXXFLAGS += $(CFLAGS) -std=c++11 -fno-common -fno-exceptions -fno-unwind-tables -fno-stack-protector

all: $(OUTFILES)

$(OBJS): | $(BUILDDIR)

$(BUILDDIR):
	mkdir -p $(BUILDDIR)


$(CXXOBJS) : $(BUILDDIR)/%.o : %.cxx Makefile
	@echo
	$(CXX) -c $(CXXFLAGS) -I. $(IINCDIR) $< -o $@

$(CPPOBJS) : $(BUILDDIR)/%.o : %.cpp Makefile
	@echo
	$(CXX) -c $(CXXFLAGS) -I. $(IINCDIR) $< -o $@

$(COBJS) : $(BUILDDIR)/%.o : %.c Makefile
	@echo
	$(CC) -c $(CFLAGS) -I. $(IINCDIR) $< -o $@

%.elf: $(OBJS) $(LDSCRIPT)
	@echo
	$(LD) $(COBJS) $(CXXOBJS) $(CPPOBJS) $(LDFLAGS) $(LIBS) -o $@

%.bin: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O binary $< $@

%.hex: %.elf $(LDSCRIPT)
	$(OBJCOPY) -O ihex $< $@

clean:
	-rm -f -r .dep $(BUILDDIR)

# Include dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
