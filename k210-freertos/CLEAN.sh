#!/bin/bash

cd k210-freertos > /dev/null 2>&1

rm -Rf build/* > /dev/null 2>&1
rm -Rf mpy_support/build/* > /dev/null 2>&1

make -C ../micropython/mpy-cross clean -s V=1

make -C ../mklittlefs clean -s V=1

make clean

rm -f mpy_support/k210_config.h > /dev/null 2>&1

rm -Rf tmp/* > /dev/null 2>&1
rm mconf > /dev/null 2>&1
rm conf > /dev/null 2>&1
rm -f .tmpconfig
rm -f .tmpconfig_tristate
