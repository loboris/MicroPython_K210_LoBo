#!/bin/bash

cd micropython/mpy-cross
make clean

cd ../../k210-freertos #> /dev/null 2>&1
make clean

