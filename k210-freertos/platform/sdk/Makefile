#
# Makefile for MicroPython for K210 project
# Build FreeRTOS libraries
#

.PHONY:all update_mk
###################################
# USER OPTION
LIB_NAME ?= sdk
CROSS_COMPILE ?=
OUTPUT_DIR :=build/
MAKE_OPT ?=

####################################
# Optional options
PLATFORM ?=
####################################
CUR_DIR_ADDR := $(shell pwd)/
SDK_DIR_NAME := kendryte-freertos-sdk
SDK_DIR_ADDR := $(CUR_DIR_ADDR)$(SDK_DIR_NAME)/

MK_VALUE :=
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/bsp/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/bsp/syscalls/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/arch/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/drivers/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/conf/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/portable/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/utils/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/hal/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/posix/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/posix/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/fatfs/source/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/lwip/src/include/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/lwip/src/include/netif/ppp/polarssl/"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/mbedtls/src/include/"
MK_VALUE +="liba-sdk := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/freertos/libfreertos.a"

MK_VALUE +="liba-drivers := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/drivers/libdrivers.a"
MK_VALUE +="liba-bsp := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/bsp/libbsp.a"
MK_VALUE +="liba-hal := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/hal/libhal.a"
MK_VALUE +="liba-posix := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/posix/libposix.a"
MK_VALUE +="liba-fatfs := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"third_party/fatfs/libfatfs.a"
MK_VALUE +="liba-lwipcore := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"third_party/lwip/liblwipcore.a"
MK_VALUE +="liba-mbedtls := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"third_party/mbedtls/libmbedtls.a"

MK_VALUE +="output-file += "'$$(liba-sdk)'
MK_VALUE +="platform-lds += "$(CUR_DIR_ADDR)$(SDK_DIR_NAME)"/lds/kendryte.ld"
####################################

#all: compile update_mk
all: compile

compile:
	$(info =====[       SDK: compile using cmake)
	@mkdir -p build
	@cd build && cmake ../$(SDK_DIR_NAME)/ -DTOOLCHAIN=$(subst bin/,bin,$(dir $(CROSS_COMPILE))) && make -s V=1 $(MAKE_OPT)

update_mk:
	$(info =====[       SDK: update mk file)
	$(shell printf '%s\n' $(MK_VALUE) > mk)

clean:
	$(info =====[       SDK: clean)
	@rm -rf build
	@rm -f mk
	
	
