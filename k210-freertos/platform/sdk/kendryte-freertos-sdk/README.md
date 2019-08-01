Kendryte K210 SDK with FreeRTOS
======

This SDK is for Kendryte K210 which contains FreeRTOS support. <br> 
Ported from [Kendryte `kendryte-freertos-sdk`](https://github.com/kendryte/kendryte-freertos-sdk)

### Includes changes to make the SDK work better with MicroPython

---

### Modifications

* Drivers
  * **uarths**: added - `uarths_baudrate` global variable used by `uarths_init` function
  * **uart**: enable using external uart irq handler, added functions `uart_get_driver(handle_t file)`, `uart_get_handle(void *driver)`
  * **i2c**: some changes and new functions
  * **spi**: many changes, some new functions, much _improved_ **slave** support
  * **gpiohs**: some changes for better interrupt handling
  * **gpio**: added some missing functions
  * **timer**: added some functions
  * **dmac**: `dma_completion_isr`, do not assert on wrong int status, but provide the interrupt status in global variable to be used by caller function 
* **syslog**: various changes
  * print log time in micro seconds
  * make syslog level configurable during run time
  * make syslog colors configurable during run time
  * add syslog mutex
  * enable filtering of non-printable characters
* FreeRTOS **`tasks`**:
  * lot of changes for better multiprocessor support
  * some useful functions added or enabled
  * all notification functions uses 64-bit values
* FreeRTOS **`heap_4`**: added functions `pvPortRealloc` & `pvPortCalloc`
* **filesystem**: added setting file time from external function (`get_fattime(void)` function), enabled `f_mkfs()` function, default code page set to 850
* **lwip**: added SHA256, enabled PPP support, enabled IGMP, thread priority set to 12
  * SNTP: added function `set_rtc_time_from_seconds(time_t seconds)` to set RTC, enabled support for DNS names
* added reading 64-bit CSR, function: `read_csr64(reg)`
* other small changes for better functionality and no compiler warnings

---

_**Full list of modified files:**_

* lib/arch/include/
  * `encoding.h`
* lib/bsp/
  * `entry_user.c`
  * `dump.c`
  * `printf.c`
  * device/
    * `dmac.cpp`
    * `gpiohs.cpp`
    * `i2c.cpp`
    * `rtc.cpp`
    * `spi.cpp`
    * `timer.cpp`
    * `uart.cpp`
* lib/drivers/src/storage/
  * `sdcard.cpp`
* lib/utils/include/
  * `syslog.h`
* lib/freertos/
  * conf/
    * `FreeRTOSConfig.h`
  * include/
    * `devices.h`
    * `filesystem.h`
    * `hal.h`
    * `FreeRTOS.h`
    * `osdefs.h`
    * `portabe.h`
    * `task.h`
    * kernel/
      * `driver.hpp`
      * `object.hpp`
  * kernel/
    * `devices.cpp`
    * storage/
      * `filesystem.cpp`
  * portable/
    * `heap_4.c`
    * `portmacro.h`
* lib/hal/
  * `uarths.c`
  * include/
    * `spi.h`
    * `uarths.h`
* third_party/
  * fatfs/
    * source/
      * `ffconf.h`
  * lwip/
    * src/
      * `Filelist.cmake`
      * api/
        * `sockets.c`
      * include/
        * `lwipopts.h`
        * lwip/
          * `opt.h`
          * apps/
            * sntp_opts.h`
        * netif/
          * ppp/
            * `ppp_opts.h`

---
