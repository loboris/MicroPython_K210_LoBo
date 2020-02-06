# Kboot

**Kboot** is a small applications (< 8KB) which enables loading and executing applications from any K210 SPI Flash location.

## Features

* Load and execute K210 application stored at any location in SPI Flash.
* Application to be loaded is chosen from the list of available applications in **Kboot configuration sector**.
* Multiple criteria application validity check is performed when selecting the application.
* If no valid application is found **default application** is loaded and executed.
* The **interactive mode** can be enabled in which, based on the specific **Pin** state, the user can choose which application to load and execute from the list of found applications.
<br>

## K210 Boot process when using **kboot**

* After reset, the **1st part** (stage #0) of **Kboot** application is loaded by **K210 ROM** to SRAM address **`0x80000000`** and executed.
* SPI Flash driver is initialized in **XiP** mode.
* The **2nd part** (stage #1) of **Kboot** application is loaded to high SRAM address **`0x805E0000`** and the execution is continued fram that address.
* If allowed by configuration, **Boot Pin** state is checked and saved<br>If Boot Pin is activated (pulled to GND) **interactive mode** is enabled.
* **Configuration sector** is scanned for valid applications.<br>Following actions are performed:
  * Configuration entry ID is checked
  * Application Flash address is cheched for valid range
  * Application size is checked for valid range
  * If **Size** flag is set, application size is checked agains the acctual size written in application's _Flash block_.
  * Application flags are saved
  * Application _Flash block_ **<sup>*</sup>** is checked for valid format
  * If **CRC32** flag is set, application's crc32 is calculated and checked
  * If **SHA256** flag is set, application's SHA256 hash is calculated and checked
* If no valid application is found, the **backup** configuration sector is scanned.
* If all checks passes, the application is marked as **valid**.
* If **not in interactive mode**, configuration sector scan is terminated as soon as the first valid, flagged as **active** application is found and that application is loaded and executed.
* If in **interactive mode**, all configuration sector entries are scanned and the user is prompted to select which one to load and execute.
* If **no valid application** is found, the **default application** is loaded and executed
* If the **default application** fails the application test, the system is **halted**

> **<sup>*</sup>** each application stored in SPI Flash has the following format:
>
> | Byte offset | Size | Name | Content |
> | ---: | ---: | :---: | :--- |
> | `0` | `1` | AES_FLAG | AES cipher flag, for use with **Kboot** must be `0` |
> | `1` | `4` | APP_SIZE | Application code size |
> | `5` | `APP_SIZE` | APP_CODE | Application code |
> | `APP_SIZE + 5` | `32` | SHA_HASH | Application SHA256 hash |
>
> All applications flashed with **ktool.py** or **kflash.py** have such format.

<br>

## K210 SPI Flash layout

The following SPI Flash layout must be used when using **Kboot*

| From | To | Length | Comment |
| ---: | ---: | ---: | :--- |
| `0x00000000` | `0x00000FFF` | 4K | 1st part of **Kboot** application, **stage #0** code |
| `0x00001000` | `0x00003FFF` | 12K | 2nd part of **Kboot** application, **stage #1** code |
| `0x00004000` | `0x00004FFF` | 4K | **main** boot configuration sector |
| `0x00005000` | `0x00005FFF` | 4K | **backup** boot configuration sector |
| `0x00006000` | `0x0000FFFF` | 40K | reserved, user data etc. |
| `0x00010000` | `DEF_APP_END` | --- | **default application** code |
| >`DEF_APP_END` | `FLASH_END` | --- | user area, application(s) code, file system(s), user data etc. |

<br>

## **Boot configuration** sector

Boot configuration used by **Kboot** is stored in one SPI Flash sector (4KB) at fixed Flash address (`0x00004000`).<br>
One **backup** configuration sector is also used for security reasons, stored in one SPI Flash sector (4KB) at fixed Flash address (`0x00005000`).<br>

The configuration sector consists of **`8`** _application entries_ occupying **`32`** bytes each.<br>
After the last application entry the **config flags** entry is placed, one 32bit value.<br>

Configuration sector layout:

| Offset | To | Length | Comment |
| ---: | ---: | ---: | :--- |
| `0x000` | `01F` | `32` | application entry #0 |
| `0x020` | `03F` | `32` | application entry #1 |
| `0x040` | `05F` | `32` | application entry #2 |
| `0x060` | `07F` | `32` | application entry #3 |
| `0x080` | `09F` | `32` | application entry #4 |
| `0x0A0` | `0BF` | `32` | application entry #5 |
| `0x0C0` | `0DF` | `32` | application entry #6 |
| `0x0E0` | `0FF` | `32` | application entry #7 |
| `0x100` | `103` | `4` | **config flags** |
| `0x104` | `11F` | `4` | reserved |
| `0x120` | `11F` | `4` | not used, user data |
<br>

The format of each _application entry_ in configuration sector is as follows:

| Offset | Length | Comment |
| ---: | ---: | :--- |
| `0` | `4` | Configuration entry **ID** (bits 4-31) + entry **flags** (bits 0-4) |
| `4` | `4` | Application address in SPI Flash |
| `8` | `4` | Application size in SPI Flash. This is the size of the application's **`.bin`** file |
| `12` | `4` | Application's CRC32 value (32bits) |
| `16` | `16` | Null terminated application name or description |

_Notes:_<br>
Configuration entry **ID** must have a value of **`0x5AA5D0C0`** for entry to be recognized as valid.<br>
Application addres must be in range **`0x10000`** ~ **`0x800000`** ( 64KB ~ 8MB ).<br>
Application size must be in range **`0x4000`** ~ **`0x300000`** ( 16KB ~ 3MB ).<br>
Application CRC32 value is used only if **CRC32** flag is set.<br><br>

If **config flags** at offset `0x100` in **configuration sector** is set to configuration entry **ID** (**`0x5AA5D0C0`**), **interractive mode** will be disabled, **boot Pin* will not be checked and nothing will be printed during the boot process.
Configuration entry **flags**:

| Bit | Comment |
| :---: | :--- |
| `0` | **Active** flag, if set the application will be loaded and executed.<br>If multiple entries have **active** flag set, the first one will be loaded ad executed |
| `1` | **CRC32** flag, if set the application's CRC32 value will be calculated and compared with the value in the configuration entry |
| `2` | **AES256** flag, if set the application's AES256 hash value will be calculated and compared with the value stored in flash after the application code (`SHA_HASH`) |
| `3` | **Size** flag, if set the application size specified in configuration entry must match the application size present in application's _Flash block_ |

**_Warning:_** all 32-bit values in the configuration sector entries must be written in **big-endian** format.

<br>

## Default application

If in the boot process **no valid application** was found in the **configuration sector**, the **default application** is loaded and executed.<br>
**_The default application should try to check if loading it was **intended** or it was result of an error condition, in which case it should try to correct the issue._**

<br>

## Building the applications to be used with _Kboot_

Nothing special must be done to build the applications which are going to be used with **Kboot**.<br>
All applications built with Kendryte **Standalone SDK** or **FreeRTOS SDK** should run without issues.<br>

The application can check if the **Kboot** system is used by reading the 2nd SPI Flash sector (at address **`0x1000`**).<br>
At offset `0x09` the `Kboot` **id string** is positioned: **`Kboot_v1.4.1`** (version mumbers may be different).<br>

```
[00001000] 00 B0 1F 00 00 6F 00 40 01 4B 62 6F 6F 74 5F 76 31 2E 34 2E 31 00 00 00 00 19 71 86 FC A2 F8 A6 
[00001020] F4 CA F0 CE EC D2 E8 D6 E4 DA E0 5E FC 62 F8 66 F4 6A F0 6E EC F3 27 40 F1 81 27 17 27 00 00 23 
```

_See the note about sector data!_

<br><br>

## Usage

Flashing the application(s) to SPI Flash and manipulating the boot **configuration sector** should be performed from the user application.<br>
Accessing SPI flash from K210 application is quite easy and reliable.



### Example

**Application firmware update**

* The condition to update the firmware is detected by the application
* Load the **boot configuration** sector, check its integrity and check the flash address and size of the currently running application
* Flash the new firmware to the **not used** SPI Flash area. The new firmware can be downloaded from remote server using WiFi or GSM, loaded from SD Card, etc ...<br>Use the **application block** format described above.<br>The application can be flashed **without** SHA256 hash, in that case do not set the **`SHA256`** flag.
* Calculate new firmware's CRC32 and/or SHA256 hash if needed.
* Flash the current **main** boot configuration sector to the **backup** boot configuration sector and check it.<br>It will be used by **Kboot** in case the **main** boot configuration sector which we are going to write is corrupted.
* Update (add or change entry) in the boot configuration sector with the new firmware information, set the new firmware as active and the old firmware as inactive and flash the configuration sector.
* Reset (reboot) the system to start the new firmware.


### Important Notes

When reading sectors written with **ktool.p<** or **kflash.py** from SPI Flash in **normal** mode (not using **XiP** mode), the sector data may look corrupted.<br>
The reason is that all sector data are written with **swapped** 32-bit values (32-bit **big endian** format is used).<br>
To correct this, you should **swap endianess** of all 32-bit values in the sector, something like this:.<br>

```
uint8_t buf4[4];
for (int k=0; k<4096; k+=4) {
    buf4[0] = sector[k+0];
    buf4[1] = sector[k+1];
    buf4[2] = sector[k+2];
    buf4[3] = sector[k+3];
    sector[k+0] = buf4[3];
    sector[k+1] = buf4[2];
    sector[k+2] = buf4[1];
    sector[k+3] = buf4[0];
}
```
