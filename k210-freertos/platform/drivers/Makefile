#
# Makefile for MicroPython for K210 project
#

.PHONY:all update_mk compile
CUR_DIR_ADDR := $(shell pwd)/

###############################################################################
# USER OPTION
LIB_NAME ?= drivers
OUTPUT_DIR := build/
CROSS_COMPILE ?= 
MK_VALUE :=
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"."
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"include"
MK_VALUE +="liba-sipeed_drivers += "$(CUR_DIR_ADDR)$(OUTPUT_DIR)$(LIB_NAME)".a"
MK_VALUE +="output-file += "'$$(liba-sipeed_drivers)'
OUTPUT_STATIC_LIB := $(LIB_NAME).a
CXX := $(CROSS_COMPILE)c++
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

MAKE_OPT ?=
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
###############################################################################
# IMPORT MK FILE
# sinclude mk MICROPYTHON NEED OTHER MK,SO I INCLUDE THE MK WHERE IS IN ROOT DIR
sinclude ../mk
###############################################################################
# IMPORT PLATFORM OPTIONS
sinclude $(PLATFORM).mk
###############################################################################

define sub_clean
	$(MAKE) -C $(1) clean -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM);
endef

define sub_make
	$(info =====[       Drivers: $(dir $(1)))
	$(MAKE) -C $(basename $(patsubst %/,%,$(patsubst ./%,%,$(dir $(1))))) all $(VERBOSE) $(MAKE_OPT) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM)

endef

define update_sub_mk
	if [ "$(dir $(1))" != "./" ] ; \
	then \
		$(MAKE) -C $(1) update_mk -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	fi;
endef
###############################################################################

GET_SUBDIRS1 := $(shell find . -maxdepth 1 -type d)
GET_SUBDIRS2 := $(basename $(patsubst ./%,%,$(GET_SUBDIRS1)))
SUBDIRS := $(GET_SUBDIRS2)

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


# AllDirs := $(shell ls -R | grep '^\./.*:$$' | awk '{gsub(":","");print}') .
FILE_MAKEFILE := $(foreach n,$(SUBDIRS) , $(wildcard $(n)/Makefile))

all: compile

$(OUTPUT_STATIC_LIB):$(CXX_OBJ) $(SRC_C_OBJ)
	$(Q)$(AR) rcs -o $(OUTPUT_DIR)$@ $(addprefix $(OUTPUT_DIR), $(SRC_C_OBJ) $(CXX_OBJ))

compile: $(OUTPUT_STATIC_LIB)
	$(info =====[       Drivers: compile $(OUTPUT_STATIC_LIB))
# $(foreach n,$(FILE_MAKEFILE),$(call sub_make,$(n)))

update_mk:
	$(info =====[       Drivers: update mk file)
	$(foreach n,$(FILE_MAKEFILE),$(call update_sub_mk, $(dir $(n))))
	$(shell printf '%s\n' $(MK_VALUE) > mk)
	$(shell printf '%s\n' $(foreach nn,$(foreach n,$(SUBDIRS) , $(wildcard $(n)/mk)),"-include "$(CUR_DIR_ADDR)$(nn)) >> mk)
clean:
	$(info =====[       Drivers: clean)
	@rm -f mk
	@rm -rf build
	$(foreach n,$(FILE_MAKEFILE),$(call sub_clean, $(dir $(n))))

