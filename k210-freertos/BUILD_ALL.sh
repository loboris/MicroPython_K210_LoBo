#!/bin/bash

cd k210-freertos > /dev/null 2>&1

mkdir -p ../firmware/sqlite
mkdir -p ../firmware/twotasks
mkdir -p ../firmware/ota
cp -f ktool.py ../firmware
cp -f kflash.py ../firmware
cp -f MPyTerm.py ../firmware

# ----------------------------------------------------
echo "=== BUILD default firmware ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi

./BUILD.sh -c .config.default -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi
cp -f MicroPython.bin ../firmware
cp -f MicroPython.kfpkg ../firmware
# ----------------------------------------------------

# ----------------------------------------------------
echo "=== BUILD sqlite firmware  ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi

./BUILD.sh -c .config.sqlite -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi
cp -f MicroPython.bin ../firmware/sqlite
cp -f MicroPython.kfpkg ../firmware/sqlite
# ----------------------------------------------------

# ----------------------------------------------------
echo "=== BUILD TwoTask firmware ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi

./BUILD.sh -c .config.twotasks -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi
cp -f MicroPython.bin ../firmware/twotasks
cp -f MicroPython.kfpkg ../firmware/twotasks
# ----------------------------------------------------

# ----------------------------------------------------
echo "=== BUILD OTA firmware     ==="
./CLEAN.sh > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi

./BUILD.sh -c .config.ota -j16 > /dev/null 2>&1
if [ $? -ne 0 ]; then
echo "=== ERROR ==="
exit 1
fi
cp -f MicroPython.bin ../firmware/ota
cp -f MicroPython.kfpkg ../firmware/ota
# ----------------------------------------------------

# ----------------------------------------------------
echo ""
echo "=== Creating firmwares archive ==="

cd ../firmware
rm -f *.zip
zip MicroPython_firmwares.zip -9 -r *
cd ../k210-freertos
# ----------------------------------------------------

echo "=== Finished ==="

./CLEAN.sh

cp -f .config.default .config

