#
# Makefile for MicroPython for K210 project
#

.PHONY:all update_mk compile
CUR_DIR_ADDR := $(shell pwd)/

###############################################################################
# USER OPTION
LIB_NAME ?= platform
CROSS_COMPILE ?= 
MK_VALUE := 
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"include" 
OUTPUT_STATIC_LIB := $(LIB_NAME).a

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

define sub_clean
	$(MAKE) -C $(1) clean -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM);
endef

define sub_make
	$(MAKE) -C $(basename $(patsubst %/,%,$(patsubst ./%,%,$(dir $(1))))) all $(VERBOSE) CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM)

endef

define update_sub_mk
	if [ "$(dir $(1))" != "./" ] ; \
	then \
		$(MAKE) -C $(1) update_mk -s V=1 CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
	fi;
endef

sinclude mk

GET_SUBDIRS1 := $(shell find . -maxdepth 1 -type d)
GET_SUBDIRS2 := $(basename $(patsubst ./%,%,$(GET_SUBDIRS1)))
SUBDIRS := $(GET_SUBDIRS2)

SRC_C := $(wildcard *.c))
SRC_CPP := $(wildcard *.cpp)

# AllDirs := $(shell ls -R | grep '^\./.*:$$' | awk '{gsub(":","");print}') .
FILE_MAKEFILE := $(foreach n,$(SUBDIRS) , $(wildcard $(n)/Makefile))

all: compile

compile:
	$(info =====[    Platform: compile)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_make,$(n)))

update_mk:
	$(info =====[    Platform: update mk file)
	$(foreach n,$(FILE_MAKEFILE),$(call update_sub_mk, $(dir $(n))))
	$(shell printf '%s\n' $(MK_VALUE) > mk)
	$(shell printf '%s\n' $(foreach nn,$(foreach n,$(SUBDIRS) , $(wildcard $(n)/mk)),"-include "$(CUR_DIR_ADDR)$(nn)) >> mk)

clean:
	rm -f mk
	$(info =====[    Platform: clean)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_clean, $(dir $(n))))

