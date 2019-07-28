# MicroPython for Kendryte K210


> This MicroPython port is now in a stable phase, but some bugs and issues can be expected, please report<br>
> All standard MicroPython functionality and modules are implemented, as well as advanced thread support, file system support, display module, nettwork, enhanced K210 peripheral support ...<br>
> *Modules providing support for still unsupported K210 peripherals will be implemented soon*
> 

***

> For discussion about this MicroPython port please wisit the [dedicated Forum](https://loboris.eu/forum/forumdisplay.php?fid=17)

***

[Wiki pages](https://github.com/loboris/MicroPython_K210_LoBo/wiki) are provided with detailed imformation about usage, classes, methods ... (not yet complete)

***

_**If you find this project useful, you can contribute by making a donation**_.&nbsp;&nbsp;[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/BLovosevic)

***

This implementation was based on Sipeed [**MaixPy**](https://github.com/sipeed/MaixPy), but it deverged from it and is now a completely independent project.<br>

### Main features:

* Based on **kendryte-freertos-sdk**, modified to include some needed features not yet implemented and enhenced drivers support
* **MicroPython core** based on **latest release** build from [main Micropython repository](https://github.com/micropython/micropython)<br>Unchanged except for the *thread* support and other changes needed to work with 64-bit RISC-V.
* `BUILD.sh` script is provided to make building MicroPython firmware as easy as possible
* Running **two** independent **Micropython instances** on **two K210 cores** is supported.<br>Rich set of functions for data exchange between instances is provided.
* Refactored **_thread** module with many new features, based on my MicroPython for ESP32 thread module.<br>
Rich set of threads related methods, including inter-thread notifications and messages.
* Full support for **LittleFS** as default file system on internal Flash.<br>
LittleFS **file system image** can be prepared on host and flashed to the board.<br>
SPIFFS is also supported (with directories support) and can be selected when building.
* Full support for Fat32 file system on **external SD Card**
* Full **file timestamp** support for LittleFS, SPIFFS and Fat32
* improved **utime** module, all time related values are handled as 64-bit values
* **SRAM** buffer is provided with content preserved after reset, accessible via `machine.mem_xxx` methods
* **uart** module, compatible with the uart module on my ESP32 port<br>buffered, interrupt based input with callbacks on events
* **Pin** module, compatible with the Pin module on my ESP32 port
* **i2c** module, **master** & **slave**, compatible with the i2c module on my ESP32 port<br>SSD1306 module provided as MicroPython frozen module
* **spi** module, **master** & **slave**, compatible with the spi module on my ESP32 port with added slave support
* **WS2812** (neopyxel) support, using **spi** module
* **ymodem** module for file transfer to/from K210 board using ymodem protocol is provided
* **uhashlib** and **ucryptolib** using K210 hardware AES are implemented
* **Display** module ported from my MicroPython for ESP32 display module<br>Rich set of drawing methods, direct write or **framebuffer**, *jpeg*, *bmp* and *raw* image format<br>Flexible fonts support (fixed width and proportional) with user fonts loadable from file (online *Font creator* provided)
* Full **network** support for **WiFi** (based on ESP8266/ESP8285) and **GSM** modules.
  * My own **ESP8266/ESP8285 firmware** is provided with enhanced features and OTA firmware update support
  * **PPPoS** support for GSM modules
  * full **SMS** support for GSM modules with collbacks on SMS receive
  * flexible and feature rich **AT command** method is provided for both WiFi and GSM modules
  * Full MicroPython **socket module** compatibility, with some additional features. FTP Server and Web Server examples are available.
  * **urequests** module (written in C) supporting all http request methods, http and https ( SSL/TLS, on WiFi, only)
  * Full featured **mqtt module** (written in C) with callbacks on all events
  * **FTP Server** and **Web Server** provided as **frozen modules**
* **usqlite3** module, data base on internal flash or SD Card, in-memory database supported
* **Boot menu**, invoked by external pin, is provided which can help solve some issues.<br>Options to prevent loading of *boot.py* and/or *main.py*, force format of the internal file system, load the default configuation ...
* Many **configuration options** can be configured directly from MicroPython (saved on Flash):
  * running two MicroPython instances
  * heap and stack sizes
  * using PyStack
  * default CPU frequency
  * default REPL baudrate
  * boot menu pin
  * default log level
  * using ansi colors
  * MicroPython VM divisor
* **pye**, full screen file editor (as MicroPython **frozen module**) is provided
* **MPyTerm**, serial terminal emulator specially designed for MicroPython is provided.<br>
Included are commands to syncronize MicroPython time from PC, change the REPL baudrate, transfer files to/from MicroPython filesystem (including transfering whole directories).<br>Fast block file transfer is used, protected with CRC.
* **Eclipse** project files included.<br>To include it into Eclipse goto:<br> `File->Import->General->Existing Projects into Workspace`<br>`Select root directory` [select *MicroPython_K210_LoBo* directory]<br>`Finish`.<br>Rebuild index<br>Execute `Index->Freshen All Files`.
* Many modules from my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki) port are already ported, porting some others is planned.
* _More to come soon ..._

<br>

***

This MicroPython port was tested on **Sipeed's MAIX** boards.<br>
Here are some links to the distributors:

[Seed](https://www.seeedstudio.com/sipeed)<br>
[AnalogLamb](https://www.analoglamb.com/product/sipeed-maix-bit-for-risc-v-aiiot-with-lcdcamera-development-board/)<br>
[Banggood](https://www.banggood.com/search/sipeed.html?sbc=1) (prices to high)<br><br>
It should also work on [Kendryte KD233](https://www.analoglamb.com/product/dual-core-risc-v-64bit-k210-ai-board-kendryte-kd233/), probably with some modifications, but it was not tested.

---

For build instructions see the [Wiki](https://github.com/loboris/MicroPython_K210_LoBo/wiki/build).

---
