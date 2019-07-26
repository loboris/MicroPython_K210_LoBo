#!/bin/bash

TOOLS_VER=ver20190410.id
TOOLCHAIN_ARCHIVE_NAME="kendryte-toolchain_v2a.tar.xz"

VERBOSE=""
J_OPTION=""

# Get arguments
POSITIONAL_ARGS=()
arg_key="$1"

while [[ $# -gt 0 ]]
do
	arg_key="$1"
	case $arg_key in
	    -v|--verbose)
	    VERBOSE+="yes"
	    shift # past argument
	    ;;
		-V|--VERBOSE)
	    VERBOSE="yesyes"
	    shift # past argument
	    ;;
	    *)    # unknown option
	    arg_opt="$1"
	    if [ "${arg_opt:0:2}" == "-j" ]; then
	        J_OPTION=${arg_opt}
	    else
	        POSITIONAL_ARGS+=("$1") # save it in an array for later
	    fi
	    shift # past argument
	    ;;
	esac
done
#set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters


cd k210-freertos > /dev/null 2>&1

# ==============================================
# Check if the toolchain toolchain is downloaded 
# ==============================================
if [ ! -d "${PWD}/../kendryte-toolchain" ]; then
    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    rm -f ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    echo "Downloading kendryte-toolchain, please wait ..."
    wget https://loboris.eu/sipeed/${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "'kendryte-toolchain' download FAILED"
        exit 1
    fi
    echo "Unpacking kendryte-toolchain"
    tar -xf ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "unpacking 'kendryte-toolchain' FAILED"
        exit 1
    fi
    rm -f ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1

    cd k210-freertos
    echo "'kendryte-toolchain' prepared, ready to build"
    sleep 2
fi

# ==========================
# Check for toolchain update 
# ==========================
if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
    make clean  > /dev/null 2>&1
	make -C ../mklittlefs > /dev/null 2>&1

    echo "Kendryte toolchain needs to be upgraded."
    echo "Removing old tools version and cleaning the build..."
    # Remove directories from previous version
    rm -rf ${PWD}/../kendryte-toolchain/ > /dev/null 2>&1
    rmdir ${PWD}/../kendryte-toolchain > /dev/null 2>&1

    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    rm -f ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    echo "Downloading new version of kendryte-toolchain, please wait ..."
    wget https://loboris.eu/sipeed/${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "'kendryte-toolchain' download FAILED"
        exit 1
    fi
    echo "Unpacking kendryte-toolchain"
    tar -xf ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "unpacking 'kendryte-toolchain' FAILED"
        exit 1
    fi
    rm -f ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1

    cd k210-freertos
    if [ ! -f "${PWD}/../kendryte-toolchain/${TOOLS_VER}" ]; then
        echo "wrong 'kendryte-toolchain' version, cannot continue!"
        exit 1
    else
        echo "'kendryte-toolchain' prepared, ready to build"
    fi
    sleep 2
fi

# ===========================================
# === Start building MicroPython firmware ===
# ===========================================

# === build mklfs =======================
make -C ../mklittlefs > /dev/null 2>&1
if [ $? -eq 0 ]; then
echo "===[ 'mklfs' created ]==="
else
echo "===[ ERROR compiling 'mklfs' ]==="
fi
sleep 1
# =======================================

MICROPY_VER=$( cat mpy_support/mpconfigport.h | grep "MICROPY_PY_LOBO_VERSION" | cut -d'"' -s -f 2 )
export MICROPY_VERSION="MaixPy-FreeRTOS_LoBo v${MICRO_PI_VER}"

MICROPY_FLASH_START=$( cat mpy_support/mpconfigport.h | grep "MICRO_PY_FLASHFS_START_ADDRESS" | cut -d'(' -s -f 2 | cut -d')' -s -f 1 | head -n 1 )
FLASH_START_ADDRES=$(( ${MICROPY_FLASH_START} ))

FS_USED=$(cat mpy_support/mpconfigport.h | grep "#define MICRO_PY_FLASHFS_USED" | cut -d'(' -f 2)
if [ "${FS_USED}" == "MICRO_PY_FLASHFS_LITTLEFS)" ]; then
	# Do not compile spiffs if not used
	mv third_party/spiffs/Makefile third_party/spiffs/Makefile.notused > /dev/null 2>&1
else
	if [ -f third_party/spiffs/Makefile.notused ]; then
		mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
	fi
fi

echo "===[ BUILDING MicroPython FIRMWARE ]==="

make update_mk > /dev/null
if [ $? -eq 0 ]; then
echo "."
else
echo "===[ First pass ERROR ]==="
exit 1
fi
make update_mk > /dev/null
if [ $? -eq 0 ]; then
echo "."
else
echo "===[ First pass ERROR ]==="
exit 1
fi

export CROSS_COMPILE=${PWD}/../kendryte-toolchain/bin/riscv64-unknown-elf-
export PLATFORM=k210
export MAKE_OPT="${J_OPTION}"

if [ "${VERBOSE}" == "yes" ]; then
make all
elif [ "${VERBOSE}" == "yesyes" ]; then
export VERBOSE=""
make all
else
make all > /dev/null
fi

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

# =========================================
# === Create kfpkg package ================
# =========================================
echo "===[ Creating 'MaixPy.kfpkg' ]==="
rm -f *.img > /dev/null 2>&1
echo "{" > flash-list.json
echo "    \"version\": \"0.1.0\"," >> flash-list.json
echo "    \"files\": [">> flash-list.json
echo "        {">> flash-list.json
echo "            \"address\": 0,">> flash-list.json
echo "            \"bin\": \"MaixPy.bin\",">> flash-list.json
echo "            \"sha256Prefix\": true">> flash-list.json
if [ -f "${PWD}/../mklittlefs/maixpy_lfs.img" ]; then
cp ${PWD}/../mklittlefs/maixpy_lfs.img .
echo "        },">> flash-list.json
echo "        {">> flash-list.json
echo "            \"address\": ${FLASH_START_ADDRES},">> flash-list.json
echo "            \"bin\": \"maixpy_lfs.img\",">> flash-list.json
echo "            \"sha256Prefix\": false">> flash-list.json
echo "        }">> flash-list.json
else
echo "        }">> flash-list.json
fi
echo "    ]">> flash-list.json
echo "}">> flash-list.json

rm -f *.kfpkg > /dev/null 2>&1
if [ -f maixpy_lfs.img ]; then
zip MaixPy.kfpkg -9 flash-list.json MaixPy.bin maixpy_lfs.img > /dev/null
else
zip MaixPy.kfpkg -9 flash-list.json MaixPy.bin > /dev/null
fi

if [ $? -eq 0 ]; then
echo "===[ kfpkg created ]==="
else
echo "===[ ERROR creating kfpkg ]==="
exit 1
fi
rm -f *.img > /dev/null 2>&1
rm -f *.json > /dev/null 2>&1
# =========================================

echo
echo "------------------------------------------------"
../kendryte-toolchain/bin/riscv64-unknown-elf-size MaixPy
echo "------------------------------------------------"
echo
echo "============================"
echo "====== Build finished ======"
echo "            version: ${MICROPY_VER}"
echo " Firmware file size: ${ALLIGNED_SIZE}"
echo " Flash FS starts at: ${FLASH_START_ADDRES}"
echo "============================"
else
	
echo "===================="
echo "==== Build ERROR ==="
echo "===================="
fi


if [ -f third_party/spiffs/Makefile.notused ]; then
	mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
fi
