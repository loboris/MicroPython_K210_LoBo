# MicroPython for Kendryte K210


To flash the pre-built firmware to your K210 board, run (in this directory):

```
./kflash.py -p /dev/ttyUSB0 -b 2000000 -t MaixPy.bin
```

Change */dev/ttyUSB0* to the port used to connect to the your board if needed.<br>
`MaixPy.bin` is the default firmware name. Other firmware names can be used.

Two firmwares are provided, one with sqlite3 compiled and one without it.<br>

`MaixPy.kfpkg` is also provided which contains prebuilt LittleFS internal file syste.<br>
To flash it, just replace `MaixPy.bin` with `MaixPy.fpkg`.

