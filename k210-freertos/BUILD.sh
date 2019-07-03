#!/bin/bash

TOOLS_VER=ver20190409.id

cd k210-freertos > /dev/null 2>&1

# ==============================================
# Check if the toolchain toolchain is downloaded 
# ==============================================
if [ ! -d "${PWD}/../kendryte-toolchain" ]; then
    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    rm -f kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    echo "Downloading kendryte-toolchain, please wait ..."
    wget https://loboris.eu/sipeed/kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "'kendryte-toolchain' download FAILED"
        return 1
    fi
    echo "Unpacking kendryte-toolchain"
    tar -xf kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "unpacking 'kendryte-toolchain' FAILED"
        return 1
    fi
    rm -f kendryte-toolchain_v2.tar.xz > /dev/null 2>&1

    cd k210-freertos
    echo "'kendryte-toolchain' prepared, ready to build"
    sleep 2
fi

# ==========================
# Check for toolchain update 
# ==========================
if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
    make clean  > /dev/null 2>&1

    echo "Kendryte toolchain needs to be upgraded."
    echo "Removing old tools version and cleaning build..."
    # Remove directories from previous version
    rm -rf ${PWD}/../kendryte-toolchain/ > /dev/null 2>&1
    rmdir ${PWD}/../kendryte-toolchain > /dev/null 2>&1

    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    rm -f kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    echo "Downloading new version of kendryte-toolchain, please wait ..."
    wget https://loboris.eu/sipeed/kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "'kendryte-toolchain' download FAILED"
        return 1
    fi
    echo "Unpacking kendryte-toolchain"
    tar -xf kendryte-toolchain_v2.tar.xz > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "unpacking 'kendryte-toolchain' FAILED"
        return 1
    fi
    rm -f kendryte-toolchain_v2.tar.xz > /dev/null 2>&1

    cd k210-freertos
    if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
        echo "wrong 'kendryte-toolchain' version, cannot continue!"
        return 1
    else
        echo "'kendryte-toolchain' prepared, ready to build"
    fi
    sleep 2
fi

# ======================
# === Start building ===
# ======================

MICROPY_VER=$(cat mpy_support/mpconfigport.h | grep "MICROPY_PY_LOBO_VERSION" | cut -d'"' -f 2)
export MICROPY_VERSION="MaixPy-FreeRTOS_LoBo v${MICRO_PI_VER}"

FS_USED=$(cat mpy_support/mpconfigport.h | grep "#define MICRO_PY_FLASHFS_USED" | cut -d'(' -f 2)
if [ "${FS_USED}" == "MICRO_PY_FLASHFS_LITTLEFS)" ]; then
	# Do not compile spiffs
	mv third_party/spiffs/Makefile third_party/spiffs/Makefile.notused
else
	if [ -f third_party/spiffs/Makefile.notused ]; then
		mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
	fi
fi

make update_mk
make update_mk

export CROSS_COMPILE=${PWD}/../kendryte-toolchain/bin/riscv64-unknown-elf-
export PLATFORM=k210

make all

# ===============================================================================
# For some weird reason the compiled binary sometimes won't run
# It looks it is related to how the binary is created from elf in makefile.
# Running the 'objcopy' here again with the following options, solves the problem
# ===============================================================================

if [ $? -eq 0 ]; then
FILESIZE=$(stat -c%s MaixPy.bin)
ALLIGNED_SIZE=$(( (((${FILESIZE} / 4096) * 4096)) + 8192 ))
END_ADDRESS=$(( ${ALLIGNED_SIZE} + 2147483648 ))

mv MaixPy.bin MaixPy.bin.bkp
../kendryte-toolchain/bin/riscv64-unknown-elf-objcopy --output-format=binary --file-alignment 4096 --gap-fill 0xFF --pad-to ${END_ADDRESS} MaixPy MaixPy.bin

#../kendryte-toolchain/bin/riscv64-unknown-elf-objdump -S  --disassemble MaixPy > MaixPy.dump

echo "==== Build finished. ===="
fi

if [ -f third_party/spiffs/Makefile.notused ]; then
	mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
fi
