# MicroPython for Kendryte K210


Three prebuilt firmwares are provided:

* `default` default configuration, no sqlite module, one MicroPython task; in this directory
* `sqlite`  default configuration, sqlite module included, one MicroPython task; in `sqlite` directory
* `twotasks`  no KPU (8 MB SRAM used), sqlite module included, two MicroPython task; in `twotasks` directory
* `default` default configuration, no sqlite module, one MicroPython task, **OTA enabled**; in `ota` directory

To flash the pre-built firmware to your K210 board, run (in this directory):

```
./kflash.py -p /dev/ttyUSB0 -b 2000000 -t MicroPython.bin
```

Change */dev/ttyUSB0* to the port used to connect to the board if needed.<br>
You can replace `MicroPython.bin` with `sqlite/MicroPython.bin`, `twotasks/MicroPython.bin` or `ota/MicroPython.bin`to flash another firmware.

`MicroPython.kfpkg` is also provided which contains prebuilt LittleFS internal file system.<br>
To flash it, just replace `MicroPython.bin` with `MicroPython.kfpkg`.


## Flashing OTA firmware

**OTA firmware** is suposed to run with **KBoot** bootloader.<br>
_Kboot bootloader_ is included in `ota/MicroPython.kfpkg`, when flashing for the **first time**, always use **`MicroPython.kfpkg`**, not a `MicroPython.bin`!<br>

After the _Kboot bootloader_ is flashed, you can later flash only the MicroPython binary using the command:

```
./kflash.py -p /dev/ttyUSB0 -a 65536 -b 2000000 -t MicroPython.bin
```
