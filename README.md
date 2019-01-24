# MicroPython for Kendryte K210

<br>

This implementation is based on [**MaixPy**](https://github.com/sipeed/MaixPy)<br>
Based on kendryte-freertos-sdk it brings many new features:

* MicroPython core based on latest build from [main Micropython repository](https://github.com/micropython/micropython), unchanged except for the thread support.
* Refactored **_thread** module with many new features, based on my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki/thread)
* Full support for SPIFFS file system on internal Flash(with directories support) and Fat32 file system on external SD Card
* Full filesystem timestamp support for both SPIFFS and Fat32
* Display module ported from my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki/display)
* **Eclipse** project files included. To include it into Eclipse goto File->Import->Existing Projects into Workspace->Select root directory->[select *MicroPython_K210_LoBo* directory]->Finish. **Rebuild index**.
* Portimg most of the modules from my [MicroPython for ESP32](https://github.com/loboris/MicroPython_ESP32_psRAM_LoBo/wiki) is planned.
* ...

<br>

Full Wiki pages with detailed instructions for build and usage will be available soon.

---

### How to Build

---

## Clone the MicroPython repository

```
git clone https://github.com/loboris/MicroPython_K210_LoBo.git
```

_You may want to use **--depth** clone option to limit the git history to only the latest commit._<br>
_The download will be much smaller and faster._

```bash
git clone --depth 1 https://github.com/loboris/MicroPython_K210_LoBo.git
```
<br>

You can also download the repository as a zip file (using `Clone or download` button on the first repository page) and unpack it to some directory.<br>
You will not be able to use `git pull` to update the repository, but otherwise the building process is the same.
<br><br>

## Build the MicroPython firmware
<br>

*Kendryte toolchain will be automatically downloaded and unpacked on* **first run** *of* **BUILD.sh** *script*
*kendryte-freertos-sdk is included in the repository*


**Change the working directory to MicroPython_K210_LoBo/k210-freertos**

To build the MicroPython firmware, run:
```
./BUILD.sh
```

To flush the built firmware to your K210 board, run:
```
./kflash.py -p /dev/ttyUSB0 -b 2000000 -t MaixPy.bin
```

Change */dev/ttyUSB0* to the port used to connect to the your board if needed.

---

```
I (188995) w25qxx_init: manuf_id:0xc8, device_id:0x17


 __  __              _____  __   __  _____   __     __ 
|  \/  |     /\     |_   _| \ \ / / |  __ \  \ \   / /
| \  / |    /  \      | |    \ V /  | |__) |  \ \_/ / 
| |\/| |   / /\ \     | |     > <   |  ___/    \   /  
| |  | |  / ____ \   _| |_   / . \  | |         | |   
|_|  |_| /_/    \_\ |_____| /_/ \_\ |_|         |_|
------------------------------------------------------

I (228135) [MAIXPY]: Stack:  min: 8065
I (232617) [MAIXPY]:  Pll0: freq: 780000000
I (237391) [MAIXPY]:  Pll1: freq: 159714285
I (242165) [MAIXPY]:   Cpu: freq: 390000000
I (246938) [MAIXPY]: Flash:   ID: [0xc8:0x17]
I (535090) [VFS_SPIFFS]: Flash VFS registered.

MaixPy-FreeRTOS by LoBo v1.0.1
------------------------------
MicroPython 76eeec7-dirty on 2019-01-24; Sipeed_board with Kendryte-K210
Type "help()" for more information.
>>> 
>>> 
>>> import machine, uos, utime, _thread
>>> def test():
...     cnt = 0
...     machine.initleds()
...     while 1:
...         machine.setled(machine.LEDR, False)
...         machine.setled(machine.LEDR, False)
...         if (cnt % 2):
...             machine.setled(machine.LEDR, True)
...         else:
...             machine.setled(machine.LEDB, True)
...         cnt += 1
...         utime.sleep_ms(500)
...
>>> 
>>> th = _thread.start_new_thread("TestLED", test, ())
>>> _thread.list()

Total system run time: 880.221 s, Tasks run time: 880.221 s, numTasks=5
MicroPython threads:
-----------------------------------------------------------------------------------------------------
ID(handle) Proc             Name     State  Stack MaxUsed PyStack    Type Priority Run time (s)   (%)
-----------------------------------------------------------------------------------------------------
2148287848    0          TestLED   running  16016    1232    2048  PYTHON        8        0.354  0.04
2148402616    0       MainThread   running  31600    1256    4096    MAIN        8        4.000  0.45

FreeRTOS tasks running:
-------------------------------------------------------------------------------
ID(handle) Proc             Name     State MinStack Priority Run time (s)   (%)
-------------------------------------------------------------------------------
2148402616    0          mp_task   Running    30344       15        4.000  0.45
2148287848    0          TestLED     Ready    14784        8        0.354  0.04
2148355008    0             IDLE     Ready     7320        0      875.613 99.48
2148357096    1             IDLE     Ready     7416        0      880.183100.00
2148367744    1    hal_tick_task   Blocked     7432        0        0.008  0.00

>>> 
>>> 
>>> 
>>> sd = uos.VfsSDCard()
>>> uos.mount(sd, '/sd')
>>> uos.listdir('/sd')
['test1.avi', 'test.txt', 'test1.txt', 'Picture', 'Video', 'Audio', 'VideoCall', 'Money.wav', 'Wall1.raw', 'TEST_F~1.TXT', 'test_syscalls.tx
t', 'test_filesystem.txt']
>>> 
>>> uos.statvfs('/sd')
(4096, 4096, 1936436, 1925131, 1925131, 0, 0, 0, 0, 255)
>>> 

```
