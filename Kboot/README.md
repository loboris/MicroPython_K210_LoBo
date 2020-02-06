# Kboot

**Kboot** is a small applications which enables loading and executing applications from any K210 SPI Flash location.

It can be used to implement firmware upgrade (OTA) or to load different (multiple) applications based on some criteria.

Interactive mode is also provided which enables the user to select which application to load and execute from the list of stored applications.

---
> More details about **Kboot** are available in the [**Kboot.md**](https://github.com/loboris/Kboot/blob/master/Kboot.md).
---

<br>

## How to build

Clone the repository or download and unpack the repository zip file.

**The repository contains all the tools and sources needed to build `Kboot`.**

The same build prerequisites must be satisfied as for building any other K210 application.<br>

Default application (**`config.bin`**, which runs if no configured aplications are found), default configuration sector (**`config.bin`**) and 3 example applications are provided.<br>

Example applications:<br>
**MicroPython.bin**, built with **_FreeRTOS SDK_** is expected to be found at Flash address **`0x00080000`** (512 KB),<br>
**dvp_ov.bin**, **_Standalone SDK_** example, is expected to be found at Flash address **`0x00280000`** (2.5 MB)<br>
**maixpy.bin**, built with **_Standalone SDK_**, is expected to be found at Flash address **`0x00280000`** (2.5 MB)<br>

You can flash the default application executting:

```console
./ktool.py -p /dev/ttyUSB0 -a 65536 -b 2000000 -t default.bin
```

You can flash one of example aplications executing:

```console
./ktool.py -p /dev/ttyUSB0 -a 524288 -b 2000000 -t MicroPython.bin
./ktool.py -p /dev/ttyUSB0 -a 2621440 -b 2000000 -t dvp_ov.bin
./ktool.py -p /dev/ttyUSB0 -a 2621440 -b 2000000 -t maixpy.bin
```

### Build from source

* Change the working directory to the **`build`** directory.
* A simple build script **`BUILD.sh`** is provided which builds the **`Kboot`** application.
* Simply execute `BUILD.sh`
* **`kboot.kfpkg`** package will be created which includes `bootloader_lo.bin` and `bootloader_hi.bin` binaries and the default configuration.

### Flash to K210 board

**`kboot.kfpkg`** can be flashed to the K210 using the included [**`ktool.py`**](https://github.com/loboris/ktool).<br>
**_Note:_** Do not use standard **`kflash.py`**, **kboot** binaries requires flashing **4KB aligned blocks**, which is not supported by standard `kflash.py`.

<br>

### Examples

Build the **`Kboot`** package:

```console
boris@UbuntuMate:/home/kboot/build$ ./BUILD.sh

 ===========================
 === Building bootloader ===
 ===========================

=== Running 'cmake'
=== Running 'make'
Scanning dependencies of target kendryte
[ 25%] Linking C static library libkendryte.a
[ 25%] Built target kendryte
Scanning dependencies of target bootloader_lo
[ 75%] Building C object CMakeFiles/bootloader_lo.dir/src/bootloader_lo/main.c.obj
[ 75%] Building ASM object CMakeFiles/bootloader_lo.dir/src/bootloader_lo/crt.S.obj
[100%] Linking C executable bootloader_lo
Generating .bin file ...
[100%] Built target bootloader_lo

=== Finished
---------------------------------------------------
   text	   data	    bss	    dec	    hex	filename
    496	    112	      8	    616	    268	bootloader_lo
---------------------------------------------------

=== Running 'cmake'
=== Running 'make'
Scanning dependencies of target kendryte
[ 14%] Linking C static library libkendryte.a
[ 14%] Built target kendryte
Scanning dependencies of target bootloader_hi
[ 28%] Building ASM object CMakeFiles/bootloader_hi.dir/src/bootloader_hi/crt.S.obj
[ 71%] Building C object CMakeFiles/bootloader_hi.dir/src/bootloader_hi/main.c.obj
[ 71%] Building C object CMakeFiles/bootloader_hi.dir/src/bootloader_hi/fpioa.c.obj
[ 71%] Building C object CMakeFiles/bootloader_hi.dir/src/bootloader_hi/gpiohs.c.obj
[ 85%] Building C object CMakeFiles/bootloader_hi.dir/src/bootloader_hi/sha256.c.obj
[100%] Linking C executable bootloader_hi
Generating .bin file ...
[100%] Built target bootloader_hi

=== Finished
--------------------------------------------------
   text	   data	    bss	    dec	    hex	filename
   6848	   1264	     88	   8200	   2008	bootloader_hi
--------------------------------------------------

=== Creating 'kboot.kfpkg'

--------------------------------------------------------------------
To flash the kboot package to K210 run:
./ktool.py -p /dev/ttyUSB0 -b 2000000 -t kboot.kfpkg

To flash default application run:
./ktool.py -p /dev/ttyUSB0 -a 65536 -b 2000000 -t default.bin

Default config can run applications from 512K or 2.5M flash address
Some applications are provided for testing:

To flash MicroPython (FreeRTOS SDK) at 512K, run:
./ktool.py -p /dev/ttyUSB0 -a 524288 -b 2000000 -t MicroPython.bin
To flash dvp_ov (Standalone SDK) at 2.5M, run:
./ktool.py -p /dev/ttyUSB0 -a 2621440 -b 2000000 -t dvp_ov.bin
To flash maixpy (Standalone SDK) at 2.5M, run:
./ktool.py -p /dev/ttyUSB0 -a 2621440 -b 2000000 -t maixpy.bin

If no app is found, default app will be run which blinks the LED(s).
--------------------------------------------------------------------

boris@UbuntuMate:/home/kboot/build$ 
```

Flash **`Kboot`** package to K210 board:

```console
boris@UbuntuMate:/home/kboot/build$ ./ktool.py -p /dev/ttyUSB0 -b 2000000 -t kboot.kfpkg
[INFO] COM Port Selected Manually:  /dev/ttyUSB0 
[INFO] Default baudrate is 115200 , later it may be changed to the value you set. 
[INFO] Trying to Enter K210 ROM ISP Mode... 
.
[INFO] Automatically detected dan/bit/trainer 

[INFO] ROM ISP detected, loading 2nd stage ISP 
[INFO] ISP loaded in 0.921s 
[INFO] Starting 2nd stage ISP at 0x805e0000 
[INFO] Wait For 0.1 second for ISP to Boot 
[INFO] 2nd stage ISP ok 
[INFO] Selected Baudrate: 2000000 
[INFO] Baudrate changed, greeting with ISP again ...  
[INFO] 2nd stage ISP ok 
[INFO] Initialize K210 SPI Flash 
[INFO] Flash initialized successfully 
[INFO] Flash ID: 0xC86018, unique ID: 384130323209302A, size: 16 MB 
[INFO] Extracting KFPKG ...  
[INFO] Writing bootloader_lo.bin to Flash address 0x00000000 
[INFO] Flashing firmware block with SHA suffix 
Programming BIN: |=========================================================================================================| 100.0% 
[INFO] Flashed 645 B [1 chunks of 4096B] (00000000~00000FFF) in 0.137s 
[INFO] Writing bootloader_hi.bin to Flash address 0x00001000 
[INFO] Flashing firmware block with SHA suffix 
Programming BIN: |=========================================================================================================| 100.0% 
[INFO] Flashed 8149 B [2 chunks of 4096B] (00001000~00002FFF) in 0.255s 
[INFO] Writing config.bin to Flash address 0x00004000 
Programming DATA: |=========================================================================================================| 100.0% 
[INFO] Flashed 4096 B [1 chunks of 4096B] (00004000~00004FFF) in 0.132s 
[INFO] Writing config.bin to Flash address 0x00005000 
Programming DATA: |=========================================================================================================| 100.0% 
[INFO] Flashed 4096 B [1 chunks of 4096B] (00005000~00005FFF) in 0.129s 
[INFO] Rebooting...
 
--- forcing DTR inactive
--- forcing RTS inactive
--- Miniterm on /dev/ttyUSB0  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---

K210 bootloader by LoBo v.1.4.1

* Find applications in MAIN parameters
    0: ' dvp_ov example', @ 0x00280000, size=77248, app_size=1679808, App ok, NOT ACTIVE
    1: '    MicroPython', @ 0x00080000, size=1744896, app_size=1789952, App ok, ACTIVE
    2: '         maixpy', @ 0x00280000, size=1681856, app_size=1679808, App ok, NOT ACTIVE

Select the application number to load [ 0, 1, 2, d=default ] ? 1

* Loading app from flash at 0x00080000 (1789952 B)
* Starting at 0x80000000 ...

M (37678) [K210_MAIN]: Default flash configuration set

RISC-V Kendryte --------------------------------------------- 
 _    _   _____   _   ______    ___   ___   ______   _     _  
( )  / ) (___  ) ( ) (  __  )  (   \ /   ) (  __  ) ( )   ( ) 
| |_/ /   ___| | | | | |  | |  | |\ \ /| | | |__| |  \ \_/ /  
|  _ )   ( ____) | | | |  | |**| | \_/ | | |  ____)   \   /   
| | \ \  | |___  | | | |__| |  | |     | | | |         | |    
(_)  \_) (_____) (_) (______)  (_)     (_) (_)         (_)    
------------------------------------------------------------- 
MicroPython-FreeRTOS by LoBo v1.11.12 (two MPY tasks)
-----------------------------------------------------

MicroPython 1.11.12 (1abe503-dirty) built on 2020-01-02; Sipeed_board with Kendryte-K210
Type "help()" for more information.
>>> 
```
