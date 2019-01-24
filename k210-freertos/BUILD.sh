#!/bin/bash

cd k210-freertos > /dev/null 2>&1

if [ ! -d "${PWD}/../kendryte-toolchain" ]; then
    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    echo "Downloading kendryte-toolchain, please wait ..."
    wget https://loboris.eu/sipeed/kendryte-toolchain.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "'kendryte-toolchain' download FAILED"
        return 1
    fi
    echo "Unpacking kendryte-toolchain"
    tar -xf kendryte-toolchain.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "unpacking 'kendryte-toolchain' FAILED"
        return 1
    fi
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    cd k210-freertos
    echo "'kendryte-toolchain' prepared, ready to build"
    sleep 2
fi


make update_mk
make update_mk
export CROSS_COMPILE=${PWD}/../kendryte-toolchain/bin/riscv64-unknown-elf-
export PLATFORM=k210
make all
