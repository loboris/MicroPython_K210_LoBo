#!/bin/bash

machine=Linux
uname_full="$(uname -a)"
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=MacOS;;
    *)          machine=Linux
esac

echo "=====[ BUILD: Building on ${uname_full}"

# Get build arguments 

TOOLS_VER=ver20190410.id
if [ "${machine}" == "MacOS" ]; then
TOOLCHAIN_ARCHIVE_NAME="kendryte-toolchain_v2a_OSX.tar.xz"
else
linplatform="$(uname -i)"
linprocessor="$(uname -p)"
linmachine="$(uname -m)"
if [ "${linplatform}" == "armv7l" ] || [ "${linprocessor}" == "armv7l" ] || [ "${linmachine}" == "armv7l" ]; then
TOOLCHAIN_ARCHIVE_NAME="kendryte-toolchain_v2a_armhf.tar.xz"
elif [ "${linplatform}" == "aarch64" ] || [ "${linprocessor}" == "aarch64" ] || [ "${linmachine}" == "aarch64" ]; then
TOOLCHAIN_ARCHIVE_NAME="kendryte-toolchain_v2a_aarch64.tar.xz"
else
TOOLCHAIN_ARCHIVE_NAME="kendryte-toolchain_v2a.tar.xz"
fi
fi

VERBOSE_BUILD=""
J_OPTION=
CREATE_DUMP=""
RUN_MENUCONFIG=""
USE_CONFIG_FILE=""
CREATE_FS_IMAGE="yes"

# Get arguments
POSITIONAL_ARGS=()
arg_key="$1"

while [[ $# -gt 0 ]]
do
    arg_key="$1"
    case $arg_key in
        -v|--verbose)
        VERBOSE_BUILD+="yes"
        shift # past argument
        ;;
        -V|--VERBOSE)
        VERBOSE_BUILD="yesyes"
        shift # past argument
        ;;
        -m|--menuconfig)
        RUN_MENUCONFIG="yes"
        shift # past argument
        ;;
        -d|--dump)
        CREATE_DUMP="yes"
        shift # past argument
        ;;
        -n|--nofsimage)
		CREATE_FS_IMAGE=""
        shift # past argument
        ;;
        -c|--config)
        USE_CONFIG_FILE="$2"
        shift # past argument
        shift # past value
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

# =============================================
# Check if the Kendryte toolchain is downloaded 
# =============================================
if [ ! -d "${PWD}/../kendryte-toolchain" ]; then
    cd ..
    rm -f kendryte-toolchain.tar.xz > /dev/null 2>&1
    rm -f ${TOOLCHAIN_ARCHIVE_NAME} > /dev/null 2>&1
    echo "Downloading kendryte-toolchain (${TOOLCHAIN_ARCHIVE_NAME}), please wait ..."
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


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

# ===========================================
# === Start building MicroPython firmware ===
# ===========================================

# === build mconf ==============================================
if [ ! -f "mconf" ] || [ ! -f "conf" ]; then
    echo "=====[ BUILD: compiling 'menuconfig' files"
    mkdir -p ../menuconfig/build
    cd ../menuconfig/build
    cmake .. #> /dev/null 2>&1
    if [ $? -eq 0 ]; then
        make  > /dev/null 2>&1
        if [ $? -eq 0 ]; then
        cp mconf ../../k210-freertos > /dev/null 2>&1
        cp conf ../../k210-freertos > /dev/null 2>&1
        fi
    fi
    cd ../../k210-freertos

    if [ ! -f "mconf" ] || [ ! -f "conf" ]; then
        echo "=====[ BUILD: ERROR, 'menuconfig' files not compiled!"
        exit 1
    fi
    echo "=====[ BUILD: OK"
    sleep 1
fi
# ==============================================================

# ==== Select config file if requested ============================================
if [ "${USE_CONFIG_FILE}" != "" ]; then
    echo "=====[ BUILD: use '${USE_CONFIG_FILE}' as config file"
    if [ -f "${USE_CONFIG_FILE}" ]; then
        rm -f mpy_support/k210_config.h > /dev/null 2>&1
        cp -f .config .config.old > /dev/null 2>&1
        cp -f ${USE_CONFIG_FILE} .config > /dev/null 2>&1
        ./conf --silentoldconfig mpy_support/k210_config.h Kconfig > /dev/null 2>&1
        if [ $? -ne 0 ]; then
            echo "=====[ BUILD: error creating 'k210.config.h'"
            if [ -t 1 ] ; then
                # stdout is a terminal
                ./mconf Kconfig
            else
                # stdout isn't a terminal
                ./mconf Kconfig > $(tty)
            fi
            if [ $? -ne 0 ]; then
                echo "=====[ BUILD: error running 'mconf'"
                exit 1
            fi
            cp -f .config .config.old > /dev/null 2>&1
            cp -f .config ${USE_CONFIG_FILE} > /dev/null 2>&1
            ./conf --silentoldconfig mpy_support/k210_config.h Kconfig > /dev/null 2>&1
            if [ $? -ne 0 ]; then
                echo "=====[ BUILD: error creating 'k210.config.h'"
                exit 1
            fi
        fi
        echo "=====[ BUILD: Configuration files created."
    else
        echo "=====[ BUILD: ERROR, '${USE_CONFIG_FILE}' not found!"
    fi
fi
# =================================================================================


if [ "${RUN_MENUCONFIG}" == "yes" ]; then
    # ==== Run menuconfig if requested or needed ==============================
    echo "=====[ BUILD: Running 'menuconfig'"
    rm -f mpy_support/k210_config.h > /dev/null 2>&1
    ./mconf Kconfig
    if [ $? -ne 0 ]; then
        echo "=====[ BUILD: error running 'mconf'"
        exit 1
    fi
    cp -f .config .config.old > /dev/null 2>&1
    ./conf --silentoldconfig mpy_support/k210_config.h Kconfig > /dev/null 2>&1
    if [ $? -ne 0 ]; then
        echo "=====[ BUILD: error creating 'k210.config.h'"
        exit 1
    fi
    echo "=====[ BUILD: Configuration files created."
    echo ""
    echo "=====[ BUILD: Run 'BUILD.sh' again to build the MicroPython firmware"
    exit 0
    # =========================================================================
else
    # ==== Create 'k210.config.h' if not created ==================================
    if [ ! -f "mpy_support/k210_config.h" ]; then
        echo "=====[ BUILD: Creating 'k210.config.h'"
        rm -f .config.old > /dev/null 2>&1
        ./conf --silentoldconfig mpy_support/k210_config.h Kconfig > /dev/null 2>&1
        if [ $? -ne 0 ]; then
            echo "=====[ BUILD: error creating 'k210.config.h'"
            ./mconf Kconfig
            if [ $? -ne 0 ]; then
                echo "=====[ BUILD: error running 'mconf'"
                exit 1
            fi
            cp -f .config .config.old > /dev/null 2>&1
            cp -f .config ${USE_CONFIG_FILE} > /dev/null 2>&1
            ./conf --silentoldconfig mpy_support/k210_config.h Kconfig > /dev/null 2>&1
            if [ $? -ne 0 ]; then
                echo "=====[ BUILD: error creating 'k210.config.h'"
                exit 1
            fi
        fi
        echo "=====[ BUILD: Configuration files created."
    fi
    # =============================================================================
fi

# === build mklfs & create image =========
if [ ! -f "../mklittlefs/mklfs" ]; then
    echo "=====[ BUILD: compiling 'mklfs'"
    make -C ../mklittlefs > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "=====[ BUILD: ok."
    else
        echo "=====[ BUILD: ERROR!"
        exit 1
    fi
    sleep 1
fi

rm -f ../mklittlefs/MicroPython_lfs.img > /dev/null 2>&1
if [ "${CREATE_FS_IMAGE}" == "yes" ]; then
    cd ../mklittlefs
    if [ "${VERBOSE_BUILD}" == "" ]; then
        ./mklfs internalfs_image MicroPython_lfs.img > /dev/null 2>&1
    else
        ./mklfs internalfs_image MicroPython_lfs.img
    fi
    if [ $? -eq 0 ]; then
        echo "=====[ BUILD: File system image created"
    else
        echo "=====[ BUILD: File system image NOT created"
    fi
    cd ../k210-freertos
fi
# =======================================

# === build mpy-cross====================
if [ ! -f "../micropython/mpy-cross/mpy-cross" ]; then
    echo "=====[ BUILD: compiling 'mpy-cross'"
    make -C ../micropython/mpy-cross ${J_OPTION} > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "=====[ BUILD: ok."
    else
        echo "=====[ BUILD: ERROR!"
        exit 1
    fi
    sleep 1
fi
# =======================================

if [ ! -f "mpy_support/k210_config.h" ]; then
    echo "=====[ BUILD: ERROR, 'mpy_support/k210_config.h' not found!"
    exit 1
fi


# === Prepare the build ===

MICROPY_OTA=$( cat mpy_support/k210_config.h | grep "CONFIG_MICROPY_USE_OTA" | cut -d' ' -s -f 3 )
MICROPY_MBOOT_PIN=$( cat mpy_support/k210_config.h | grep "CONFIG_MICRO_PY_MBOOT_PIN" | cut -d' ' -s -f 3 )
MICROPY_FW_SIZE_TYPE=$( cat mpy_support/k210_config.h | grep "CONFIG_FIRMWARE_SIZE_TYPE" | cut -d' ' -s -f 3 )
if [ "${MICROPY_FW_SIZE_TYPE}" == "0" ]; then
	MICROPY_FW_SIZE="(0x200000UL)"
elif [ "${MICROPY_FW_SIZE_TYPE}" == "1" ]; then
	MICROPY_FW_SIZE="(0x280000UL)"
elif [ "${MICROPY_FW_SIZE_TYPE}" == "2" ]; then
    MICROPY_FW_SIZE="(0x300000UL)"
else
    MICROPY_FW_SIZE="(0x380000UL)"
fi

if [ "${MICROPY_OTA}" != "" ]; then
    echo "=====[ BUILD: compiling 'kboot'"

    echo "/*" > ../Kboot/src/bootloader_hi/include/config.h
    echo " * Defined from MicroPython build" >> ../Kboot/src/bootloader_hi/include/config.h
    echo " */" >> ../Kboot/src/bootloader_hi/include/config.h
    echo "#define FIRMWARE_SIZE     ${MICROPY_FW_SIZE}" >> ../Kboot/src/bootloader_hi/include/config.h
    echo "#define BOOT_PIN          ${MICROPY_MBOOT_PIN}" >> ../Kboot/src/bootloader_hi/include/config.h
    echo "" >> ../Kboot/src/bootloader_hi/include/config.h
    cd ../Kboot/build
    ./BUILD.sh
    if [ $? -eq 0 ]; then
        echo "=====[ BUILD: ok."
    else
        cd ../../k210-freertos
        echo "=====[ BUILD: ERROR!"
        exit 1
    fi
    cd ../../k210-freertos
    cp -f ../Kboot/build/bootloader_??.bin .
    cp -f ../Kboot/build/config.bin .
fi

MICROPY_VER=$( cat mpy_support/mpconfigport.h | grep "MICROPY_PY_LOBO_VERSION" | cut -d'"' -s -f 2 )
export MICROPY_VERSION="MicroPython_K210_LoBo v${MICRO_PI_VER}"

MICROPY_FLASH_START=$( cat mpy_support/k210_config.h | grep "CONFIG_MICRO_PY_FLASHFS_START_ADDRESS" | cut -d' ' -s -f 3 )
FLASH_START_ADDRES=$(( ${MICROPY_FLASH_START} * 1024 * 1024 ))

FS_USED=$( cat mpy_support/k210_config.h | grep "CONFIG_MICROPY_FILESYSTEM_TYPE" | cut -d' ' -s -f 3 )
if [ "${FS_USED}" == "0" ]; then
    # === Do not compile spiffs if not used ===
    if [ -f third_party/spiffs/Makefile ]; then
        mv third_party/spiffs/Makefile third_party/spiffs/Makefile.notused > /dev/null 2>&1
    fi
else
    if [ -f third_party/spiffs/Makefile.notused ]; then
        mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
    fi
fi

# --------------------------------------------------------
# Select the linker script based on firmware size selected
# --------------------------------------------------------

if [ "${MICROPY_FW_SIZE_TYPE}" == "0" ]; then
    echo "=====[ BUILD: 2 MB firmware"
    cp -f platform/sdk/kendryte-freertos-sdk/lds/kendryte_2M.ld platform/sdk/kendryte-freertos-sdk/lds/kendryte.ld
elif [ "${MICROPY_FW_SIZE_TYPE}" == "1" ]; then
    echo "=====[ BUILD: 2.5 MB firmware"
    cp -f platform/sdk/kendryte-freertos-sdk/lds/kendryte_2_5M.ld platform/sdk/kendryte-freertos-sdk/lds/kendryte.ld
elif [ "${MICROPY_FW_SIZE_TYPE}" == "2" ]; then
    echo "=====[ BUILD: 3 MB firmware"
    cp -f platform/sdk/kendryte-freertos-sdk/lds/kendryte_3M.ld platform/sdk/kendryte-freertos-sdk/lds/kendryte.ld
else
    echo "=====[ BUILD: 3.5 MB firmware"
    cp -f platform/sdk/kendryte-freertos-sdk/lds/kendryte_3_5M.ld platform/sdk/kendryte-freertos-sdk/lds/kendryte.ld
fi

# --------------------------------------
# Update mk files, it must be done twice
# --------------------------------------
echo ""
echo "=====[ BUILD: UPDATE mk files"
make update_mk > /dev/null
if [ $? -ne 0 ]; then
    echo "=====[ BUILD: ERROR!"
    exit 1
fi

if [ "${VERBOSE_BUILD}" == "yes" ] || [ "${VERBOSE_BUILD}" == "yesyes" ]; then
    make update_mk
else
    make update_mk > /dev/null
fi
if [ $? -ne 0 ]; then
    echo "=====[ BUILD: ERROR!"
    exit 1
fi
echo ""

echo "==========================================="
echo "=====[ BUILD: BUILDING MicroPython FIRMWARE"
echo "==========================================="

export CROSS_COMPILE=${PWD}/../kendryte-toolchain/bin/riscv64-unknown-elf-
export PLATFORM=k210
export MAKE_OPT="${J_OPTION}"

if [ "${VERBOSE_BUILD}" == "yes" ]; then
    export BUILD_VERBOSE=0
    make all
elif [ "${VERBOSE_BUILD}" == "yesyes" ]; then
    export BUILD_VERBOSE=1
    make all
else
    export BUILD_VERBOSE=0
    make all > /dev/null
fi

# ===============================================================================
# For some weird reason the compiled binary sometimes won't run
# It looks it is related to how the binary is created from elf in Makefile.
# Running the 'objcopy' here again with the following options, solves the problem
# ===============================================================================

if [ $? -eq 0 ]; then
if [ "${machine}" == "MacOS" ]; then
    FILESIZE=$(stat -s MicroPython.bin | tr ' ' '\n' | grep st_size| cut -d= -f2)
else
    FILESIZE=$(stat -c%s MicroPython.bin)
fi
ALIGNED_SIZE=$(( (((${FILESIZE} / 4096) * 4096)) + 8192 ))
# Code in SRAM starts at 0x80000000, correct the address
END_ADDRESS=$(( ${ALIGNED_SIZE} + 2147483648 ))
# Code in SRAM starts at 0x80000000, correct the address
END_ADDRESS=$(( ${ALIGNED_SIZE} + 2147483648 ))

mv MicroPython.bin MicroPython.bin.bkp
../kendryte-toolchain/bin/riscv64-unknown-elf-objcopy --output-format=binary --file-alignment 4096 --gap-fill 0xFF --pad-to ${END_ADDRESS} MicroPython MicroPython.bin

# Get real file size
if [ "${machine}" == "MacOS" ]; then
    FILESIZE=$(stat -s MicroPython.bin | cut -d' ' -s -f 1 | cut -d'=' -s -f 2)
else
    FILESIZE=$(stat -c%s MicroPython.bin)
fi

if [ "${CREATE_DUMP}" == "yes" ]; then
    echo "=====[ BUILD: Exporting objdump"
    ../kendryte-toolchain/bin/riscv64-unknown-elf-objdump -S  --disassemble MicroPython > MicroPython.dump
fi

# =========================================
# === Create kfpkg package ================
# =========================================

if [ "${MICROPY_OTA}" != "" ]; then
    echo ""
    echo "=====[ BUILD: Creating 'MicroPython.kfpkg' (OTA)"
    rm -f *.img > /dev/null 2>&1
    echo "{" > flash-list.json
    echo "    \"version\": \"0.1.0\"," >> flash-list.json
    echo "    \"files\": [">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 0,">> flash-list.json
    echo "            \"bin\": \"bootloader_lo.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": true">> flash-list.json
    echo "        },">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 4096,">> flash-list.json
    echo "            \"bin\": \"bootloader_hi.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": true">> flash-list.json
    echo "        },">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 16384,">> flash-list.json
    echo "            \"bin\": \"config.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": false">> flash-list.json
    echo "        },">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 20480,">> flash-list.json
    echo "            \"bin\": \"config.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": false">> flash-list.json
    echo "        },">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 65536,">> flash-list.json
    echo "            \"bin\": \"MicroPython.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": true,">> flash-list.json
    echo "            \"swap\": false">> flash-list.json
    if [ -f "${PWD}/../mklittlefs/MicroPython_lfs.img" ]; then
        cp ${PWD}/../mklittlefs/MicroPython_lfs.img .
        echo "        },">> flash-list.json
        echo "        {">> flash-list.json
        echo "            \"address\": ${FLASH_START_ADDRES},">> flash-list.json
        echo "            \"bin\": \"MicroPython_lfs.img\",">> flash-list.json
        echo "            \"sha256Prefix\": false,">> flash-list.json
        echo "            \"swap\": false">> flash-list.json
        echo "        }">> flash-list.json
    else
        echo "        }">> flash-list.json
    fi
    echo "    ]">> flash-list.json
    echo "}">> flash-list.json
else
    echo ""
    echo "=====[ BUILD: Creating 'MicroPython.kfpkg'"
    rm -f *.img > /dev/null 2>&1
    echo "{" > flash-list.json
    echo "    \"version\": \"0.1.0\"," >> flash-list.json
    echo "    \"files\": [">> flash-list.json
    echo "        {">> flash-list.json
    echo "            \"address\": 0,">> flash-list.json
    echo "            \"bin\": \"MicroPython.bin\",">> flash-list.json
    echo "            \"sha256Prefix\": true,">> flash-list.json
    echo "            \"swap\": false">> flash-list.json
    if [ -f "${PWD}/../mklittlefs/MicroPython_lfs.img" ]; then
        cp ${PWD}/../mklittlefs/MicroPython_lfs.img .
        echo "        },">> flash-list.json
        echo "        {">> flash-list.json
        echo "            \"address\": ${FLASH_START_ADDRES},">> flash-list.json
        echo "            \"bin\": \"MicroPython_lfs.img\",">> flash-list.json
        echo "            \"sha256Prefix\": false,">> flash-list.json
        echo "            \"swap\": false">> flash-list.json
        echo "        }">> flash-list.json
    else
        echo "        }">> flash-list.json
    fi
    echo "    ]">> flash-list.json
    echo "}">> flash-list.json
fi

if [ "${MICROPY_OTA}" != "" ]; then
    rm -f *.kfpkg > /dev/null 2>&1
    if [ -f MicroPython_lfs.img ]; then
        zip MicroPython.kfpkg -9 flash-list.json bootloader_lo.bin bootloader_hi.bin config.bin MicroPython.bin MicroPython_lfs.img > /dev/null
    else
        zip MicroPython.kfpkg -9 flash-list.json bootloader_lo.bin bootloader_hi.bin config.bin MicroPython.bin > /dev/null
    fi
    rm -f bootloader_??.bin
    rm -f config.bin
else
    rm -f *.kfpkg > /dev/null 2>&1
    if [ -f MicroPython_lfs.img ]; then
        zip MicroPython.kfpkg -9 flash-list.json MicroPython.bin MicroPython_lfs.img > /dev/null
    else
        zip MicroPython.kfpkg -9 flash-list.json MicroPython.bin > /dev/null
    fi
fi

if [ $? -eq 0 ]; then
if [ -f "${PWD}/../mklittlefs/MicroPython_lfs.img" ]; then
        echo "=====[ BUILD: kfpkg created"
    else
        echo "=====[ BUILD: kfpkg created without FS image)"
    fi
else
    echo "=====[ BUILD: ERROR creating kfpkg"
    exit 1
fi
rm -f *.img > /dev/null 2>&1
rm -f *.json > /dev/null 2>&1
# =========================================

echo
if [ "${VERBOSE_BUILD}" == "" ]; then
    echo "---------------------------------------------------"
    ../kendryte-toolchain/bin/riscv64-unknown-elf-size MicroPython
    echo "---------------------------------------------------"
    echo
fi


echo "============================="
echo "====== Build finished ======="
echo "            version: ${MICROPY_VER}"
echo " Firmware file size: ${FILESIZE}"
MICROPY_FILE_CRC32=$(crc32 MicroPython.bin 2> /dev/null)
if [ $? -eq 0 ]; then
    echo "     Firmware CRC32: ${MICROPY_FILE_CRC32}"
    fi
    echo " Flash FS starts at: ${FLASH_START_ADDRES}"
    echo "============================="
else
    echo "===================="
    echo "==== Build ERROR ==="
    echo "===================="
fi


if [ -f third_party/spiffs/Makefile.notused ]; then
	mv third_party/spiffs/Makefile.notused third_party/spiffs/Makefile
fi
