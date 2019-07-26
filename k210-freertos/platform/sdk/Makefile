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
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/bsp/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/bsp/syscalls/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/arch/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/drivers/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/conf/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/freertos/portable/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/utils/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/hal/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/posix/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"lib/posix/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/fatfs/source/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/lwip/src/include/""\r\n"
MK_VALUE +="INC += "$(SDK_DIR_ADDR)"third_party/lwip/src/include/netif/ppp/polarssl/""\r\n"
MK_VALUE +="liba-sdk := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/freertos/libfreertos.a""\r\n"

MK_VALUE +="liba-drivers := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/drivers/libdrivers.a""\r\n"
MK_VALUE +="liba-bsp := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/bsp/libbsp.a""\r\n"
MK_VALUE +="liba-hal := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/hal/libhal.a""\r\n"
MK_VALUE +="liba-posix := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"SDK/posix/libposix.a""\r\n"
MK_VALUE +="liba-fatfs := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"third_party/fatfs/libfatfs.a""\r\n"
MK_VALUE +="liba-lwipcore := "$(CUR_DIR_ADDR)$(OUTPUT_DIR)"third_party/lwip/liblwipcore.a""\r\n"

MK_VALUE +="output-file += "'$$(liba-sdk)'"\r\n"
MK_VALUE +="platform-lds += "$(CUR_DIR_ADDR)$(SDK_DIR_NAME)"/lds/kendryte.ld""\r\n"
####################################

all: compile update_mk

compile:
	mkdir -p build
	cd build && cmake ../$(SDK_DIR_NAME)/ -DTOOLCHAIN=$(subst bin/,bin,$(dir $(CROSS_COMPILE))) && make $(MAKE_OPT)

update_mk:
	$(info =====[ update mk file ... ]=====)
	$(shell echo $(MK_VALUE) > mk)

clean:
	rm -rf build
	rm -f mk
	
	
