# Kboot

**Kboot** is a small applications (< 8KB) which enables loading and executing applications from any K210 SPI Flash location.

## Features

* load and execute K210 application stored on any location in SPI Flash
* application to be loaded is chosen from the list of available applications in **Kboot configuration sector**
* multiple criteria application validity check is performed when selecting the application
* if no valid application is found **default application** is loaded and executed
* The **interactive mode** can be enabled in which,based on the specific **Pin** state, the user can choose which application to load and execute
<br>

## Boot process

* After reset **Kboot** application is loaded by **K210 ROM** to sram address **`0x80000000`** and executed
* SPI Flash driver is initialized in **XiP** mode
* If allowed by configuration, **Boot Pin** state is checked and saved, if Boot Pin is activated **interactive mode** is enabled
* **Configuration sector** is scanned for valid applications, the following actions are performed:
  * Configuration entry ID is checked
  * Application Flash address is cheched for valid range
  * Application size is checked for valid range
  * Application flags are saved
  * Application _Flash block_ **<sup>*</sup>** is checked for valid format
  * If **CRC32** flag is set, application's crc32 is calculated and checked
  * If **SHA256** flag is set, application's SHA256 hash is calculated and checked
* If all checks passes, the application is marked as **valid**
* If **not in interactive mode**, configuration sector scan is terminated as soon as the first valid, flagged as **active** application is found and that application is loaded and executed
* If in **interactive mode**, all configuration sector entries are scanned and the user is prompted to select which one to load and execute
* If **no valid application** is found, the **default application** is loaded and executed

> **<sup>*</sup>** each application stored in SPI Flash has the following format:
>
> | Byte offset | Size | Name | Content |
> | ---: | ---: | :---: | :--- |
> | `0` | `1` | AES_FLAG | AES cipher flag, for use with **Kboot** must be `0` |
> | `1` | `4` | APP_SIZE | EApplication code size |
> | `5` | `APP_SIZE` | APP_CODE | Application code |
> | `APP_SIZE + 5` | `32` | SHA_HASH | Application SHA256 hash |
>
> All applications flashed with **Kflash** have such format.

<br>

## K210 SPI Flash layout

The following SPI Flash layout must be used when using **Kboot*

| From | To | Length | Comment |
| ---: | ---: | ---: | :--- |
| `0x00000000` | `0x0000FFFF` | 64K | **Kboot** application code |
| `0x00010000` | `0x0001FFFF` | 64K | reserved for future use |
| `0x00020000` | `0x00020FFF` | 4K | main **boot configuration** sector |
| `0x00021000` | `0x00021FFF` | 4K | backup **boot configuration** sector |
| `0x00022000` | `0x0002FFFF` | 56K | reserved, user data etc. |
| `0x00030000` | `0x0007FFFF` | 320K | **default application** code |
| `0x00080000` | `Flash end` | --- | user area, application(s) code, file system(s), user data etc. |

<br>

## **Boot configuration** sector

Boot configuration used by **Kboot** is stored in one SPI Flash sector (4KB) at fixed Flash address.<br>
One **backup** configuration sector is also used for security reasons.<br>

The configuration sector consists of **`8`** application entries occupying **`32`** bytes each + 32bit value, the configuration flags.<br>

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

The format of each application entry in configuration sector is as follows:

| Offset | Length | Comment |
| ---: | ---: | :--- |
| `0` | `4` | Configuration entry **ID** (bits 4-31) + entry **flags** (bits 0-4) |
| `4` | `4` | Application address in SPI Flash |
| `8` | `4` | Application size in SPI Flash. This is the size of the application's **`.bin`** file |
| `12` | `4` | Application's CRC32 value (32bits) |
| `16` | `16` | Null terminated application name/description |

_Notes:_<br>
Configuration entry **ID** must have a value of **`0x5AA5D0C0`** for entry to recognized as valid.<br>
Application addres must be in range **`0x80000`** ~ **`0x800000`** ( 512KB ~ 8MB ).<br>
Application size must be in range **`0x4000`** ~ **`0x300000`** ( 16KB ~ 3MB ).<br>
Application CRC32 value is used only if **CRC32** flag is set.<br><br>

If **config flags** at offset `0x100` in **configuration sector** is set to configuration entry **ID** (**`0x5AA5D0C0`**), **interractive mode** will be disabled, **boot Pin* and nothing will be printed during the boot process.
Configuration entry **flags**:

| Bit | Comment |
| :---: | :--- |
| `0` | **Active** flag, if set the application will be loaded and executed.<br>If multiple entries have **active** flag set, the first one will be loaded ad executed |
| `1` | **CRC32** flag, if set the application's CRC32 value will be calculated and compared with the value in the configuration entry |
| `2` | **AES256** flag, if set the application's AES256 hash value will be calculated and compared with the value stored in flash after the application code (`SHA_HASH`) |
| `3` | Not used, reserved |

**_Warning:_** all 32-bit values in the configuration sector entries must be written in **big-endian** format.

<br>

## Default application

If in the boot process **no valid application** was found in the **configuration sector**, the **default application** is loaded and executed.<br>
If the **default application** is loaded that usually means some error occured and the system may be considered unusable.<br>
**_The default application should try to check what was wrong and try to correct the issue._**

<br>

## Building the applications to be used with _Kboot_

Importand thing to remember is that the applications loaded and executed by **Kboot** are **NOT loaded** at the default K210 SRAM address **(`0x80000000`)**, but at address **`0x80002000`** !<br>
Therefore, when building the application, it must be **linked** for start address **`0x80002000`**.<br>

Fortunatelly, it is quite simple to do it, only a slight modification of the linker script is needed.<br>
The linker script is usually called **kendryte.ld** and it is located in the **Kendryte SDK** **_lds_** directory.<br>
The beginning of that file looks like this:
```
/*
 * The MEMORY command describes the location and size of blocks of memory
 * in the target. You can use it to describe which memory regions may be
 * used by the linker, and which memory regions it must avoid.
 */
MEMORY
{
  /*
   * Memory with CPU cache.
   *6M CPU SRAM
   */
  ram (wxa!ri) : ORIGIN = 0x80000000, LENGTH = (6 * 1024 * 1024)
  /*
   * Memory without CPU cache
   * 6M CPU SRAM
  */
  ram_nocache (wxa!ri) : ORIGIN = 0x40000000, LENGTH = (6 * 1024 * 1024)
}
```

The only entries which has to be changed are **`ORIGIN`** and **`LENGTH`**.<br>
Change:<br>
`ORIGIN = 0x80000000` to **`ORIGIN = 0x80002000`**<br>
`ORIGIN = 0x40000000` to **`ORIGIN = 0x40002000`**<br>
`LENGTH = (6 * 1024 * 1024)` to **`LENGTH = (6 * 1024 * 1024 - 0x2000)`**<br>

and build the application as usual.<br>

The beginning of the modified **kendryte.ld** should look like this:
```console
/*
 * The MEMORY command describes the location and size of blocks of memory
 * in the target. You can use it to describe which memory regions may be
 * used by the linker, and which memory regions it must avoid.
 */
MEMORY
{
  /*
   * Memory with CPU cache.
   *6M CPU SRAM
   */
  ram (wxa!ri) : ORIGIN = 0x80002000, LENGTH = (6 * 1024 * 1024 - 0x2000)
  /*
   * Memory without CPU cache
   * 6M CPU SRAM
  */
  ram_nocache (wxa!ri) : ORIGIN = 0x40002000, LENGTH = (6 * 1024 * 1024 - 0x2000)
}
```

**_When building the application also check if any part of the code depends on application start address in SRAM._**<br>
For most applications it is not the case.

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

