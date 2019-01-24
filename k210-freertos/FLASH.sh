#!/bin/bash

cd k210-freertos > /dev/null 2>&1
mate-terminal --working-directory="${PWD}" -e "./kflash.py -p /dev/ttyUSB0 -b 2000000 -t Maixpy.bin" &
