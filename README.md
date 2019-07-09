# MicroPython for Kendryte K210


> This MicroPython port is now in a stable phase, but some bugs and issues can be expected, please report<br>
> All standard MicroPython functionality and modules are implemented, as well as advanced thread support, file system support and display module.<br>
> *Modules providing support for still unsupported K210 peripherals will be implemented soon*
> 

<br>

This implementation is based on [**MaixPy**](https://github.com/sipeed/MaixPy)<br>
Based on *kendryte-freertos-sdk* (modified to include some needed features not yet implemented) it brings many new features:

* MicroPython core based on latest release build from [main Micropython repository](https://github.com/micropython/micropython), unchanged except for the thread support and other changes needed to work with 64-bit RISC-V.
* Running **two** independent Micropython instances on two K210 cores is supported.<br>Data exchange between instances is supported.
* Refactored **_thread** module with many new features, based on my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki/thread) _thread module
* Full support for **LittleFS** on internal Flash.<br>SPIFFS is also supported (with directories support) and can be selected when building.
* Full support for Fat32 file system on external SD Card
* Full filesystem timestamp support for LittleFS, SPIFFS and Fat32
* **ymodem** module for file transfer to/from K210 board using ymodem protocol is provided
* **uhashlib** and **ucryptolib** using K210 hardware AES are implemented
* **Display** module ported from my [MicroPython for ESP32]
(https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki/display) display module
* Full **network** support for **WiFi** (based on ESP8266/ESP8285) and **GSM** modules.
* **sqlite3** support
* **pye**, full screen file editor (as python frozen module) is provided
* **MPyTerm**, serial terminal emulator specially designed for MicroPython is provided.<br>
Included are commands to syncronize MicroPython time from PC, change the REPL baudrate, transfer files to/from MicroPython filesystem (including transfering whole directories).<br>Fast block file transfer is used, protected with CRC.
* **Eclipse** project files included.<br>To include it into Eclipse goto `File->Import->Existing Projects into Workspace->Select root directory->[select *MicroPython_K210_LoBo* directory]->Finish`. **Rebuild index**.
* Many modules from my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki) port are already ported, porting some others is planned.
* _More to come soon ..._

<br>

**Wiki pages** with detailed instructions for build and usage are provided.<br>_Missing documentation will be available soon._

---

### How to Build

---

## Clone the MicroPython repository

```
git clone https://github.com/loboris/MicroPython_K210_LoBo.git
```

<br>

You can also download the repository as a zip file (using `Clone or download` button on the first repository page) and unpack it to some directory.<br>
You will not be able to use `git pull` to update the repository, but otherwise the building process is the same.
<br><br>

## Build the MicroPython firmware
<br>

> _**Kendryte toolchain** will be automatically downloaded and unpacked on **first run** of **BUILD.sh** script.<br> It can take some time on slow Internet connection (~32 MB will be downloaded)._<br>
> _**kendryte-freertos-sdk** is included in the repository._


Change the working directory to **MicroPython_K210_LoBo/k210-freertos**.

To build the MicroPython firmware, run:
```
./BUILD.sh
```

To flash the built firmware to your K210 board, run:
```
./kflash.py -p /dev/ttyUSB0 -b 2000000 -t MaixPy.bin
```

Change */dev/ttyUSB0* to the port used to connect to the your board if needed.

---

```console
M (1371) [MAIXPY]: Configuration loaded from flash
M (9298) [MAIXPY]: Heaps: FreeRTOS=3840 KB (1196 KB free), MPy=2560 KB, other=379 KB

 __  __              _____  __   __  _____   __     __ 
|  \/  |     /\     |_   _| \ \ / / |  __ \  \ \   / /
| \  / |    /  \      | |    \ V /  | |__) |  \ \_/ / 
| |\/| |   / /\ \     | |     > <   |  ___/    \   /  
| |  | |  / ____ \   _| |_   / . \  | |         | |   
|_|  |_| /_/    \_\ |_____| /_/ \_\ |_|         |_|
------------------------------------------------------

MaixPy-FreeRTOS by LoBo v1.11.3
-------------------------------
MicroPython 1.11.3 (c593f64-dirty) built on 2019-07-03; Sipeed_board with Kendryte-K210
Type "help()" for more information.
>>> 
>>> help('modules')
__main__          io                sys               upip_utarfile
_thread           json              time              upysh
array             machine           ubinascii         urandom
binascii          math              ucollections      ure
board             microWebSocket    ucryptolib        uselect
builtins          microWebSrv       uctypes           usocket
cmath             microWebTemplate  uerrno            ustruct
collections       micropython       uftpserver        utime
display           network           uhashlib          utimeq
font10            os                uheapq            uzlib
font6             pye               uio               writer
framebuf          re                ujson             ymodem
freesans20        socket            uos
gc                ssd1306           upip
Plus any modules on the filesystem
>>> 
>>> 
paste mode; Ctrl-C to cancel, Ctrl-D to finish
=== import _thread, utime, machine, gc, micropython, os
=== 
=== def test(led, sleep_time=3000):
===     notif_exit = 4718
===     ld = machine.Pin(led, machine.Pin.OUT)
===     ld.value(1)
===     while 1:
===         notif = _thread.getnotification()
===         if notif == notif_exit:
===             print("[{}] Exiting".format(_thread.getSelfName()))
===             ld.value(1)
===             utime.sleep_ms(100)
===             return
===         elif notif == 777:
===             print("[{}] Forced EXCEPTION".format(_thread.getSelfName()))
===             ld.value(1)
===             utime.sleep_ms(1000)
===             zz = 234 / 0
===         utime.sleep_ms(sleep_time)
===         ld.value(0)
===         utime.sleep_ms(200)
===         ld.value(1)
=== 
=== th1 = _thread.start_new_thread("Test1", test, (machine.Pin.LEDR, 2000))
=== #utime.sleep_ms(1000)
=== th2 = _thread.start_new_thread("Test2", test, (machine.Pin.LEDG, 2500))
=== #utime.sleep_ms(1000)
=== th3 = _thread.start_new_thread("Test3", test, (machine.Pin.LEDB, 3300))
=== 
>>> 
>>> _thread.list()

Total system run time: 144.678 s, number of tasks running: 7
MicroPython threads:
-------------------------------------------------------------------------------------------------------------------
ID(handle) Proc             Name      State  Stack  Used MaxUsed PyStack   Used    Type Priority Run time (s)   (%)
-------------------------------------------------------------------------------------------------------------------
2148930632    0            Test3    running   8192  3216    3464    4096    128  PYTHON        8        0.346  0.24
2148915528    0            Test2    running   8192  3216    3464    4096    128  PYTHON        8        0.353  0.24
2148900424    0            Test1    running   8192  3216    3576    4096    128  PYTHON        8        0.363  0.25
2151627528    0       MainThread*   running  32768  2080    2664    4096     64    MAIN        8        0.709  0.49

FreeRTOS tasks running:
-------------------------------------------------------------------------------
ID(handle) Proc             Name     State MinStack Priority Run time (s)   (%)
-------------------------------------------------------------------------------
2151627528    0     main_mp_task   Running    30104       15        0.709  0.49
2148915528    0            Test2     Ready     4728        8        0.353  0.24
2148900424    0            Test1     Ready     4616        8        0.363  0.25
2148930632    0            Test3     Ready     4728        8        0.346  0.24
2152821640    0             IDLE     Ready     7416        0      142.885 98.76
2152823752    1             IDLE     Ready     7416        0        0.000  0.00
2148967032    0    hal_tick_task   Blocked     7432        1        0.001  0.00
-------------------------------------------------------------------------------
FreeRTOS heap: Size: 3932160, Free: 1210808, Min free: 1189024

>>> 
>>> machine.mpy_config()

Current MicroPython configuration:
----------------------------------
 MPy version code: 011103
Two MPy instances: False
     PyStack used: True
  MPy#1 heap size: 2560 KB
  MPy#2 heap size: 0 KB
     PyStack size: 4096 B
   MPy stack size: 32768 B
    CPU frequency: 400 MHz
    REPL baudrate: 115200 bd
     But menu pin: 17
Default log level: 2 (LOG_WARN)
       VM divisor: 32
(False, True, 2621440, 0, 4096, 32768, 400000000, 115200, 17, 2, 32)
>>> 
>>> machine.reset_reason()
(8, 'External pin reset')
>>> 
>>> 
--[mpTerm command: synctime
OK.
back to device ]--

>>> os.mkdir('/flash/www')
>>> 
--[mpTerm command: senddir mpy_support/examples/webserver/www /flash/www
Sending local file mpy_support/examples/webserver/www/index.html to /flash/www/index.html

--> 100.00%
OK, took 0.271 seconds, 4.040 KB/s
Sending local file mpy_support/examples/webserver/www/pdf-sample.pdf to /flash/www/pdf-sample.pdf

--> 100.00%
OK, took 1.884 seconds, 4.118 KB/s
Sending local file mpy_support/examples/webserver/www/pdf.png to /flash/www/pdf.png

--> 100.00%
OK, took 1.060 seconds, 4.192 KB/s
Sending local file mpy_support/examples/webserver/www/style.css to /flash/www/style.css

--> 100.00%
OK, took 0.025 seconds, 18.044 KB/s
Sending local file mpy_support/examples/webserver/www/wstest.html to /flash/www/wstest.html

--> 100.00%
OK, took 0.539 seconds, 3.858 KB/s
Sending local file mpy_support/examples/webserver/www/test.pyhtml to /flash/www/test.pyhtml

--> 100.00%
OK, took 0.044 seconds, 14.650 KB/s
Sending local file mpy_support/examples/webserver/www/python.ico to /flash/www/python.ico

--> 100.00%
OK, took 1.049 seconds, 3.991 KB/s
Sending local file mpy_support/examples/webserver/www/favicon.ico to /flash/www/favicon.ico

--> 100.00%
OK, took 0.455 seconds, 4.801 KB/s
back to device ]--

>>> 
--[mpTerm command: ls /flash/www

List of directory '/flash/www/':
--------------------------------
    favicon.ico  <file>  2238  2019-07-03 14:47:24
     index.html  <file>  1122  2019-07-03 14:47:10
 pdf-sample.pdf  <file>  7945  2019-07-03 14:47:11
        pdf.png  <file>  4551  2019-07-03 14:47:14
     python.ico  <file>  4286  2019-07-03 14:47:22
      style.css  <file>   457  2019-07-03 14:47:17
    test.pyhtml  <file>   666  2019-07-03 14:47:20
    wstest.html  <file>  2128  2019-07-03 14:47:18
back to device ]--

>>> os.listdir('/flash/www')
['favicon.ico', 'index.html', 'pdf-sample.pdf', 'pdf.png', 'python.ico', 'style.css', 'test.pyhtml', 'wstest.html']
>>> 

```
