### This file is used for build example projects.

# set this will supress some warnings
set(BUILDING_SDK "yes" CACHE INTERNAL "")
# LoBo: do not show all messages
set_property(GLOBAL PROPERTY RULE_MESSAGES OFF)

# basic config
if (NOT PROJ)
    #message("FreeRTOS SDK: only compiling libraries")
    set(PROJ "Dummy")
else()
    message("PROJ = ${PROJ}")
endif ()

cmake_minimum_required(VERSION 3.0)
include(./cmake/common.cmake)

project(${PROJ})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/lib/arch/include ${CMAKE_CURRENT_LIST_DIR}/lib/utils/include)
# config self use headers
header_directories(${SDK_ROOT}/lib)

# build library first
add_subdirectory(lib SDK)
add_subdirectory(third_party)

# Do not compile project if not given
if (NOT PROJ STREQUAL "Dummy")
    add_source_files(src/${PROJ}/*.c src/${PROJ}/*.s src/${PROJ}/*.S src/${PROJ}/*.cpp)
    include(./cmake/executable.cmake)
endif ()
