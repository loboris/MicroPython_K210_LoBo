#!/bin/bash

TOOLS_VER=ver20190214.id

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

if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
    make clean  > /dev/null 2>&1

    echo "Removing old tools version and cleaning build..."
    # Remove directories from previous version
    rm -rf ${PWD}/../kendryte-toolchain/ > /dev/null 2>&1
    rmdir ${PWD}/../kendryte-toolchain > /dev/null 2>&1
    touch ${PWD}/../kendryte-toolchain/${TOOLS_VER}
    echo "Kendryte toolchain version" > ${PWD}/../kendryte-toolchain/${TOOLS_VER}

    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    echo "Downloading new version of kendryte-toolchain, please wait ..."
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
    if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
        echo "wrong 'kendryte-toolchain' version, cannot continue!"
        return 1
    else
        echo "'kendryte-toolchain' prepared, ready to build"
    fi
    sleep 2
fi

# === Start building ===

export MICROPY_VERSION="MaixPy-FreeRTOS_LoBo v1.0.5"
make update_mk
make update_mk
export CROSS_COMPILE=${PWD}/../kendryte-toolchain/bin/riscv64-unknown-elf-
export PLATFORM=k210
make all
