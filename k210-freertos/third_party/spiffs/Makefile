#
# Makefile for MicroPython for K210 project
#

.PHONY:all update_mk compile
CUR_DIR_ADDR := $(shell pwd)/

###############################################################################
# USER OPTION
LIB_NAME ?= spiffs
CROSS_COMPILE ?= 
OUTPUT_DIR := build/
MK_VALUE :="INC += "$(CUR_DIR_ADDR)"
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"spiffs/src"
#MK_VALUE +="INC += "$(CUR_DIR_ADDR)"spiffs/src/default"
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"include"
MK_VALUE +="liba-spiffs += "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"spiffs.a"
MK_VALUE +="output-file += "'$$(liba-spiffs)'
OUTPUT_STATIC_LIB := $(LIB_NAME).a
SPIFFS_SRC_ROOT_DIR := spiffs
spiffs := $(SPIFFS_SRC_ROOT_DIR)
CXX := $(CROSS_COMPILE)c++
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

BUILD_VERBOSE ?= 0
ifeq ($(BUILD_VERBOSE), 1)
	VERBOSE :=
else
	VERBOSE := -s V=1
endif

###############################################################################
# Optional options
PLATFORM ?=
###############################################################################
# COMPILE OPTIONS
CFLAGS :=
LDFLAGS :=
CXXFLAGS :=
INC :=
SRC_C :=
C_OBJ :=
CPATH :=
###############################################################################
# IMPORT MK FILE
sinclude spiffs/files.mk
sinclude ../../mk
###############################################################################
# IMPORT PLATFORM OPTIONS
sinclude $(PLATFORM).mk
###############################################################################
# SOME FUNCTIONS
define sub_clean
	$(MAKE) -C $(1) clean -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM);
endef

define sub_make
	$(info =====[      SPIFFS: $(dir $(1)))
	$(MAKE) -C $(basename $(patsubst %/,%,$(patsubst ./%,%,$(dir $(1))))) all $(VERBOSE) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM)
endef

SRC_C += $(addprefix $(CPATH)/, $(CFILES)) $(wildcard spiffs-port/*.c)
#SRC_C += spiffs-port/spiffs-port.c
C_OBJ :=  $(SRC_C:.c=.o)

INCLUDE_FLAGS := $(addprefix -I, $(INC))

CFLAGS += $(INCLUDE_FLAGS)

$(C_OBJ):%.o:%.c
	@mkdir -p $(dir $(OUTPUT_DIR)$@) 
	@$(CC)  -o $(OUTPUT_DIR)$@ -c $< $(CFLAGS)

all: compile

$(OUTPUT_STATIC_LIB):$(C_OBJ)
	$(info =====[       SPIFFS: compile $(C_OBJ))
	$(Q)$(AR) rcs -o $(OUTPUT_DIR)$@ $(addprefix $(OUTPUT_DIR), $(C_OBJ))

compile: $(OUTPUT_STATIC_LIB)
	$(info =====[      SPIFFS: compile)
# $(foreach n,$(FILE_MAKEFILE),$(call sub_make,$(n)))

update_mk:
	$(info =====[      SPIFFS: update mk file)
	$(shell printf '%s\n' $(MK_VALUE) > mk)
	$(shell printf '%s\n' $(foreach nn,$(foreach n,$(SUBDIRS) , $(wildcard $(n)/mk)),"-include "$(CUR_DIR_ADDR)$(nn)) >> mk)

clean:
	$(info =====[      SPIFFS: clean)
	@rm -f mk
	@rm -rf build

