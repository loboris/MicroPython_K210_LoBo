#!/bin/bash

cd k210-freertos > /dev/null 2>&1

cd ../micropython/mpy-cross
make clean

cd ../../mklittlefs
make clean

cd ../k210-freertos #> /dev/null 2>&1
make clean

