#
# Makefile for MicroPython for K210 project
#

.PHONY: all clean compile do_mk update_all_platform_mk
CUR_DIR_ADDR := $(shell pwd)/

###############################################################################
# USER OPTION
LIB_NAME ?= MicroPython
CROSS_COMPILE ?= 
OUTPUT_DIR := build/
MK_VALUE := "INC += ."
OUTPUT_BIN := MicroPython.bin
CC := $(CROSS_COMPILE)gcc
CXX ?= $(CROSS_COMPILE)c++
AR ?= $(CROSS_COMPILE)AR
OBJCOPY ?= $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
ifeq ("$(wildcard $(CROSS_COMPILE))/../lib/gcc/riscv64-unknown-elf/8.3.0)","")
	CRT_DIR_ADDR ?=$(dir $(CROSS_COMPILE))/../lib/gcc/riscv64-unknown-elf/8.3.0/
else
	CRT_DIR_ADDR ?=$(dir $(CROSS_COMPILE))/../lib/gcc/riscv64-unknown-elf/8.2.0/
endif
CRT_DIR_ADDR ?=$(dir $(CROSS_COMPILE))/../lib/gcc/riscv64-unknown-elf/8.3.0/

BUILD_VERBOSE ?= 0
ifeq ($(BUILD_VERBOSE), 1)
	VERBOSE :=
else
	VERBOSE := -s V=1
endif

MAKE_OPT ?=
LOG_FILE := > build.log 2>&1
###############################################################################
# Optional options
PLATFORM ?=
###############################################################################
# COMPILE OPTIONS
CFLAGS :=
LDFLAGS :=
CXXFLAGS :=
INC :=
output-file :=
platform-lds :=
###############################################################################
sinclude mk
sinclude $(PLATFORM).mk

define sub_clean
	if [ "$(1)" != " mpy_support/" ] ; \
	then \
	$(MAKE) -C $(1) clean -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	else \
	$(MAKE) -C $(1) clean_all -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	fi;
endef

define sub_make
	if [ "$(1)" != " mpy_support/" ] ; \
	then \
		$(MAKE) -C $(1) $(VERBOSE) all CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	fi;
endef

define update_sub_platform_mk
	if [ "$(dir $(1))" != "./" ] ; \
	then \
		cat $(PLATFORM).mk>$(1); \
	fi;
endef

define update_sub_mk
	if [ "$(dir $(1))" != "./" ] ; \
	then \
		$(MAKE) -C $(1) update_mk -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	fi;
endef

GET_SUBDIRS1 := $(shell find . -maxdepth 1 -type d)
GET_SUBDIRS2 := $(basename $(patsubst ./%,%,$(GET_SUBDIRS1)))
SUBDIRS := $(GET_SUBDIRS2)

AllDirs := $(shell ls -R | grep '^\./.*:$$' | awk '{gsub(":","");print}') .
PLATFORM_MK := $(foreach n,$(AllDirs) , $(wildcard $(n)/$(PLATFORM).mk))
#PLATFORM_MK_X := $(shell echo "$(PLATFORM_MK)" | sed 's/  */ /g' | sed 's/ \{1,\}/ /g' | tr -s ' ')
FILE_MAKEFILE := $(foreach n,$(SUBDIRS) , $(wildcard $(n)/Makefile))

all: do_mk update_all_platform_mk sub_compile compile $(OUTPUT_BIN) print_bin_size
.SILENT: clean do_mk update_mk update_all_platform_mk compile sub_compile compile all

SRC_C := $(wildcard *.c)
SRC_CPP := $(wildcard *.cpp)
SRC_C_OBJ := $(SRC_C:.c=.o)
CXX_OBJ := $(SRC_CPP:.cpp=.o)
INCLUDE_FLAGS := $(addprefix -I, $(INC))
CFLAGS += $(INCLUDE_FLAGS)
$(CXX_OBJ):%.o:%.cpp
	@mkdir -p $(dir $(OUTPUT_DIR))
	@$(CXX)  -o $(OUTPUT_DIR)/$@ -c $< $(INCLUDE_FLAGS) $(CXXFLAGS) -lstdc++
$(SRC_C_OBJ):%.o:%.c
	@mkdir -p $(dir $(OUTPUT_DIR))
	@$(CC)  -o $(OUTPUT_DIR)/$@ -c $< $(INCLUDE_FLAGS) $(CFLAGS) -lstdc++

SRC_C_OUTPUT_OBJ := $(addprefix $(OUTPUT_DIR), $(SRC_C_OBJ))
SRC_CXX_OUTPUT_OBJ := $(addprefix $(OUTPUT_DIR), $(CXX_OBJ))

CRT_FALGS := $(CRT_DIR_ADDR)crti.o 
CRT_FALGS += $(CRT_DIR_ADDR)crtbegin.o $(SRC_C_OUTPUT_OBJ) $(SRC_CXX_OUTPUT_OBJ)
CRT_FALGS += $(CRT_DIR_ADDR)crtend.o
CRT_FALGS += $(CRT_DIR_ADDR)crtn.o

BIN_LDFLAGS := \
		-nostartfiles \
		-static \
		-Wl,--gc-sections \
		-Wl,-static \
		-Wl,--start-group \
		-Wl,--whole-archive \
		-Wl,--no-whole-archive \
		-Wl,--end-group \
		-Wl,-EL \
		-T \
		$(platform-lds) \
		$(CRT_FALGS) \
		-Wl,--start-group \
		-lm \
		$(SRC_CXX_OUTPUT_OBJ) \
		$(output-file) \
		$(liba-libfreertos) \
		$(liba-hal) \
		-latomic \
		$(liba-drivers) \
		$(liba-bsp) \
		$(liba-hal) \
		$(liba-posix) \
		-lc \
		-lstdc++ \
		$(CXX_OBJ) \
		-Wl,--end-group \
		$(liba-fatfs) \
		$(liba-lwipcore) \
		$(liba-mbedtls) \
		$(liba-hal) \
		-lm \
		-lstdc++ \
		-lm

$(info )

$(LIB_NAME): $(output-file) $(SRC_C_OUTPUT_OBJ) $(SRC_C_OUTPUT_OBJ)
	$(info =====[ Main: linking all libraries)
	@$(CC) $(CFLAGS) $(BIN_LDFLAGS) -o $@

print_bin_size:
	$(info )
	@echo "------------------------------------------------"
	@$(SIZE) $(LIB_NAME)
	@echo "------------------------------------------------"

$(OUTPUT_BIN):$(LIB_NAME)
	$(info =====[ Main: creating MicroPython binary)
	@$(OBJCOPY) --output-format=binary $(LIB_NAME) $@

compile:$(SRC_C_OBJ) $(CXX_OBJ)

sub_compile:
	$(info =====[ Main: Compiling subdirectiries)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_make, $(dir $(n))))
	$(info =====[ Main: Compiling MicroPython)
	$(MAKE) -C mpy_support $(VERBOSE) $(MAKE_OPT) all CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM)

update_mk:
	$(info =====[ Main: update mk file)
	$(foreach n,$(FILE_MAKEFILE),$(call update_sub_mk, $(dir $(n))))
	$(shell printf '%s\n' $(MK_VALUE) > mk)
	$(shell printf '%s\n' $(foreach nn,$(foreach n,$(SUBDIRS) , $(wildcard $(n)/mk)),"-include "$(CUR_DIR_ADDR)$(nn)) >> mk)

update_all_platform_mk:
	$(info =====[ Main: update all platform mk files)
	if [ -f $(PLATFORM).mk ] ; \
	then \
	     $(foreach n,$(PLATFORM_MK),$(call update_sub_platform_mk, $(n))) \
	fi;
	
clean:
	$(info =====[ Main: clean all)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_clean, $(dir $(n))))
	@rm -f mk
	@rm -rf build
	@rm -rf $(LIB_NAME) $(OUTPUT_BIN)


