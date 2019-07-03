#!/bin/bash

# ====================================================================================
# Flash the initial firmware to the ESP8266/ESP8285
# Usage:
# ./flash.sh -t|--flashtype -m|--flashmode -ne|--noerase -bo|--bootonly -fo|--fwonly -ns|--no-stub
# All parameters are optional, default values are listed bellow

# ====================================================================================

SERIAL_PORT="/dev/ttyUSB0"
FLASH_TYPE="4MB"
FLASH_MODE="qio"
EXOPT="-b 460800"
ERASE_FLASH="yes"
ONLY_BOOT="no"
ONLY_FW="no"
PART_NUM=0
TEST_ONLY="no"
ERASE_ONLY="no"
ONLY_CRT="no"
FLASH_BLANKS=""
FW_FILE=""

#----------------
get_arguments() {
    POSITIONAL_ARGS=()
    local key="$1"

    while [[ $# -gt 0 ]]
    do
    local key="$1"
    case $key in
        -t|--flashtype)
        FLASH_TYPE="$2"
        shift # past argument
        shift # past value
        ;;
        -m|--flashmode)
        FLASH_MODE="$2"
        shift # past argument
        shift # past value
        ;;
        -p|--part)
        PART_NUM="$2"
        shift # past argument
        shift # past value
        ;;
        -P|--port)
        SERIAL_PORT="$2"
        shift # past argument
        shift # past value
        ;;
        -ne|--noerase)
        ERASE_FLASH="no"
        shift # past argument
        ;;
        -eo|--eraseonly)
        ERASE_ONLY="yes"
        shift # past argument
        ;;
        -bo|--bootonly)
        ONLY_BOOT="yes"
        shift # past argument
        ;;
        -co|--crtonly)
        ONLY_CRT="yes"
        shift # past argument
        ;;
        -fo|--fwonly)
        ONLY_FW="yes"
        shift # past argument
        ;;
        -ns|--nostub)
        EXOPT="--no-stub"
        shift # past argument
        ;;
        --test)
        TEST_ONLY="yes"
        shift # past argument
        ;;
        -fw|--fwfile)
        FW_FILE="$2"
        shift # past argument
        shift # past value
        ;;
        *)    # unknown option
        shift # past argument
        ;;
    esac
    done
    #set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters
}

# ==== get arguments ====
get_arguments "$@"

# ==== check arguments ====
if [ "${FLASH_TYPE}" != "512KB" ] &&[ "${FLASH_TYPE}" != "1MB" ] && [ "${FLASH_TYPE}" != "2MB" ] && [ "${FLASH_TYPE}" != "2MB-c1" ] && [ "${FLASH_TYPE}" != "4MB" ] && [ "${FLASH_TYPE}" != "4MB-c1" ]; then
    echo "Wrong flash type!"
    exit 1
fi
if [ "${FLASH_MODE}" != "qio" ] && [ "${FLASH_MODE}" != "qout" ] &&[ "${FLASH_MODE}" != "dio" ] &&[ "${FLASH_MODE}" != "dout" ]; then
    echo "Wrong flash mode!"
    exit 1
fi
if [ ${PART_NUM} -lt 0 ] || [ ${PART_NUM} -gt 7 ]; then
    echo "Wrong partition number!"
    exit 1
fi
if [ ${PART_NUM} -gt 1 ] && [ "${FLASH_TYPE}" == "1MB" ]; then
    echo "Partition number to big"
    exit 1
fi
if [ ${PART_NUM} -gt 0 ] && [ "${FLASH_TYPE}" == "512KB" ]; then
    echo "Partition number to big"
    exit 1
fi
if [ ${PART_NUM} -gt 3 ] && [ "${FLASH_TYPE}" == "2MB" ]; then
    echo "Partition number to big"
    exit 1
fi
if [ ${PART_NUM} -gt 3 ] && [ "${FLASH_TYPE}" == "2MB-c1" ]; then
    echo "Partition number to big"
    exit 1
fi

# ==== set write options ====
if [ "${FLASH_TYPE}" == "512KB" ]; then
WROPT="-ff 40m -fm ${FLASH_MODE} -fs 1MB"
else
WROPT="-ff 40m -fm ${FLASH_MODE} -fs ${FLASH_TYPE}"
fi

# ==== set bootloader config sector blank option ====
if [ "${FLASH_TYPE}" == "512KB" ]; then
	BOOT_CONFIG_BLANK=""
else
	BOOT_CONFIG_BLANK="0xfa000 ../bin/blank.bin"
fi

# ==== set bootloader address and file name ====
if [ "${FLASH_TYPE}" == "512KB" ]; then
FLASH_BOOT="0x00000 ../bin/bootloader_noota.bin"
else
FLASH_BOOT="0x00000 ../bin/bootloader.bin"
fi

# ==== set flash address ====
FLASH_ADDRESS=$(( (${PART_NUM} * 524288) + 4096 ))
FLASH_HALF_MB=$(( (${FLASH_ADDRESS} / 524288) % 2))
if [ ${FLASH_HALF_MB} -eq 0 ]; then
    USER_APP="1"
else
    USER_APP="2"
    if [ "${FLASH_TYPE}" == "2MB-c1" ] || [ "${FLASH_TYPE}" == "4MB-c1" ]; then
        echo "Cannot flash 1024+1024 firmware at half MB address"
        exit 1
    fi
fi
FLASH_ADDRESS_HEX=$( printf "06X" ${FLASH_ADDRESS} )

# ==== If flash is erased, blank sectors don't need to be flashed, ====
# ==== else select the correct addresses based on flash size       ====
if [ "${ERASE_FLASH}" == "no" ]; then
	if [ "${FLASH_TYPE}" == "512KB" ]; then
		FLASH_BLANKS="0x7e000 ../bin/blank.bin ${BOOT_CONFIG_BLANK}"
	else
		FLASH_BLANKS="0xfe000 ../bin/blank.bin ${BOOT_CONFIG_BLANK}"
	fi
fi

# ==== Set sector addresses and file names for the default data and CA certificate ====
if [ "${FLASH_TYPE}" == "512KB" ]; then
	FLASH_CRT="0x76000 ../bin/esp_ca_cert.bin"
	FLASH_DEF="0x7c000 ../bin/esp_init_data_default_v08.bin"
else
	FLASH_CRT="0x7b000 ../bin/esp_ca_cert.bin"
	FLASH_DEF="0xfc000 ../bin/esp_init_data_default_v08.bin"
fi

# ==== Set the firmware name, For 'dout' flash mode select ESP8285 firmware ====
if [ "${FLASH_MODE}" == "dout" ]; then
    FLASH_APP="../bin/upgrade/esp8285_AT_${USER_APP}_2.bin"
else
	if [ "${FLASH_TYPE}" == "512KB" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_0.bin"
	elif [ "${FLASH_TYPE}" == "1MB" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_2.bin"
	elif [ "${FLASH_TYPE}" == "2MB" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_3.bin"
	elif [ "${FLASH_TYPE}" == "4MB" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_4.bin"
	elif [ "${FLASH_TYPE}" == "2MB-c1" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_5.bin"
	elif [ "${FLASH_TYPE}" == "4MB-c1" ]; then
		FLASH_APP="../bin/upgrade/esp8266_AT_${USER_APP}_6.bin"
	fi
fi

# ==== check firmware file ====
if [ ! -f ${FLASH_APP} ]; then
    echo "Requested file '${FLASH_APP}' not found"
    exit 1
fi

# ==== Set maximal firmware file size, and check the firmware file ====
if [ "${FLASH_TYPE}" == "2MB-c1" ] || [ "${FLASH_TYPE}" == "4MB-c1" ]; then
    MAX_FILESIZE=983040
elif [ "${FLASH_TYPE}" == "512KB" ]; then
	MAX_FILESIZE=479232
else
    MAX_FILESIZE=495616
fi

FILESIZE=$(stat -c%s $FLASH_APP)
if [ ${FILESIZE} -ge ${MAX_FILESIZE} ]; then
    echo "  Firmware file to big (${FILESIZE} > ${MAX_FILESIZE})"
    exit 1
fi


if [ "${ONLY_FW}" == "yes" ] || [ "${ONLY_BOOT}" == "yes" ] || [ "${ONLY_CRT}" == "yes" ]; then
    ERASE_FLASH="no"
fi

if [ "${FW_FILE}" != "" ]; then
    FLASH_APP=${FW_FILE}
    FLASH_BLANKS=""
    FLASH_DEF=""
    FLASH_CRT=""
fi

echo
echo "=========================="
echo "Flashing ESP8266 firmware:"
echo "--------------------------"
echo "  Flash type: ${FLASH_TYPE}"
echo "  Flash mode: ${FLASH_MODE}"
echo "   Partition: ${PART_NUM}"
printf "     Address: 0x%06x\r\n" ${FLASH_ADDRESS}
echo "    Firmware: '${FLASH_APP}'"
echo " Erase Flash: '${ERASE_FLASH}'"
echo "================================================"
echo

if [ "${TEST_ONLY}" == "yes" ]; then
    echo
    echo "Finished."
    exit 0
fi

# Erase Flash if requested
if [ "${ERASE_FLASH}" == "yes" ]; then
    esptool.py --port ${SERIAL_PORT} --after no_reset erase_flash
    if [ $? -ne 0 ]; then
        echo
        echo "Erase flash failed!"
        exit 1
    fi
    if [ "${ERASE_ONLY}" == "yes" ]; then
        echo
        echo "Finished."
        exit 0
    fi
    echo
    sleep 1
fi

# Flash
if [ "${ONLY_BOOT}" == "yes" ]; then
    esptool.py --port ${SERIAL_PORT} ${EXOPT} --after no_reset write_flash ${WROPT} ${BOOT_CONFIG_BLANK} ${FLASH_BOOT}
elif [ "${ONLY_FW}" == "yes" ]; then
    esptool.py --port ${SERIAL_PORT} ${EXOPT} --after no_reset write_flash ${WROPT} ${BOOT_CONFIG_BLANK} ${FLASH_ADDRESS} ${FLASH_APP}
elif [ "${ONLY_CRT}" == "yes" ]; then
    esptool.py --port ${SERIAL_PORT} ${EXOPT} --after no_reset write_flash ${WROPT} ${FLASH_CRT}
else
    esptool.py --port ${SERIAL_PORT} ${EXOPT} --after no_reset write_flash ${WROPT} ${FLASH_BLANKS} ${FLASH_DEF} ${FLASH_BOOT} ${FLASH_CRT} ${FLASH_ADDRESS} ${FLASH_APP}
fi
if [ $? -ne 0 ]; then
    echo
    echo "Flash write failed!"
    exit 1
fi

echo
echo "Finished."
