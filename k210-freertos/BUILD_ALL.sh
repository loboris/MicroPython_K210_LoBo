#!/bin/bash

cd k210-freertos > /dev/null 2>&1

echo "=== BUILD default firmware ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi

./BUILD.sh -c .config.default -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi
cp -f MicroPython.bin ../firmware
cp -f MicroPython.kfpkg ../firmware

echo "=== BUILD sqlite firmware ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi

./BUILD.sh -c .config.sqlite -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi
cp -f MicroPython.bin ../firmware/sqlite
cp -f MicroPython.kfpkg ../firmware/sqlite

echo "=== BUILD TwoTask firmware ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi

./BUILD.sh -c .config.twotasks -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
exit 1
fi
cp -f MicroPython.bin ../firmware/twotasks
cp -f MicroPython.kfpkg ../firmware/twotasks


echo "=== Creating firmwares archive ==="

cd ../firmware
rm -f *.zip
zip MicroPython_firmwares.zip -9 -r *
cd ../k210-freertos

