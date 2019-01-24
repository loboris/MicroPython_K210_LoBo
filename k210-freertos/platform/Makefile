#
# Makefile for maixpy
# ports/k210-freetos/mpy_support/Makefile
#
.PHONY:all update_mk compile
CUR_DIR_ADDR := $(shell pwd)/
###############################################################################
# USER OPTION
LIB_NAME ?= platform
CROSS_COMPILE ?= 
MK_VALUE := 
MK_VALUE +="INC += "$(CUR_DIR_ADDR)"include""\n" 
OUTPUT_STATIC_LIB := $(LIB_NAME).a
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
	$(MAKE) -C $(1) clean CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM);
endef

define sub_make
	$(info =====[ $(dir $(1)) ]======)
	$(MAKE) -C $(basename $(patsubst %/,%,$(patsubst ./%,%,$(dir $(1))))) all CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM)

endef

define update_sub_mk
	if [ "$(dir $(1))" != "./" ] ; \
	then \
		$(MAKE) -C $(1) update_mk CROSS_COMPILE=$(CROSS_COMPILE) PLATFORM=$(PLATFORM); \
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
	$(info =====[ CUR_DIR $(CUR_DIR) ... ]======)

compile:
	$(info =====[ AllDirs $(AllDirs) ... ]======)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_make,$(n)))

update_mk:
	$(foreach n,$(FILE_MAKEFILE),$(call update_sub_mk, $(dir $(n))))
	$(shell echo $(MK_VALUE)$(foreach nn,$(foreach n,$(SUBDIRS) , $(wildcard $(n)/mk)),"-include "$(CUR_DIR_ADDR)$(nn)"\n") > mk)

clean:
	rm -f mk
	$(info =====[ Cleaning $(SUBDIRS) ... ]======)
	$(foreach n,$(FILE_MAKEFILE),$(call sub_clean, $(dir $(n))))

