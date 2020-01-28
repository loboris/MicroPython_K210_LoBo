TARGET = mklfs

CC ?= gcc
AR ?= ar
SIZE ?= size

SRC += $(wildcard *.c)
OBJ := $(SRC:.c=.o)
DEP := $(SRC:.c=.d)
ASM := $(SRC:.c=.s)

override CFLAGS += -Os
ifdef WORD
override CFLAGS += -m$(WORD)
endif
override CFLAGS += -I.
override CFLAGS += -std=c99 -Wall -pedantic
override CFLAGS += -Wextra -Wshadow -Wjump-misses-init
# Remove missing-field-initializers because of GCC bug
override CFLAGS += -Wno-missing-field-initializers


all: $(TARGET)

asm: $(ASM)

size: $(OBJ)
	$(SIZE) -t $^

ifdef QUIET
	@./$< | sed -n '/^[-=]/p'
else
	./$<
endif

-include $(DEP)

mklfs: $(OBJ)
	$(CC) $(CFLAGS) $^ $(LFLAGS) -o $@

%.a: $(OBJ)
	$(AR) rcs $@ $^

%.o: %.c
	$(CC) -c -MMD $(CFLAGS) $< -o $@

%.s: %.c
	$(CC) -S $(CFLAGS) $< -o $@

clean:
	@rm -f $(TARGET)
	@rm -f $(OBJ)
	@rm -f $(DEP)
	@rm -f $(ASM)
