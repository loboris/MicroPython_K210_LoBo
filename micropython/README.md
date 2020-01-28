
***

This is the clone of the main [MicroPython repository](https://github.com/micropython/micropython).<br><br>
It is mostly unchanged, except for the following files:<br>

* `py/mpthread.h` & `py/modthread.c`<br>completelly rewritten thread support
* `py/obj.h`<br>changes needed for correct interpretation of floats for `MICROPY_OBJ_REPR_C` and `MICROPY_OBJ_REPR_D` for RISC-V 64bit processor
* `py/mpprint.c`<br>changes needed for usin `ll` for 64-bit integers (`MICROPY_OBJ_REPR_C`)
* `py/runtime.c`, `py/builtin.h`, `py/gc.c`, `py/gc.h`, `py/mpstate.h`<br> small changes to enable support for two MicroPython instances running on two processors
* `py/modsys.c`<br> small changes to enable support for two MicroPython instances running on two processors<br>**`sys.path`, `sys.argv` _and_ `sys.modules` _are now FUNCTIONS:_ `sys.path()`, `sys.argv()` _and_ `sys.modules()`**
* `py/modmicropython.c`<br> added PyStack info in `mem_info` function
* `py/compile.c`, `persistentcode.c`<br> small changes to address some errors when compiling without any native emmiter enabled
* `py/machine_mem.c`, `py/machine_mem.h`<br> added support for 64-bit memory objects
* `extmod/modframebuf.c`<br>added some functions (circle, fill_circle)
* `extmod/modlwip.c`<br>renamed to `extmod/modlwip.c.orig` to prevent compiling as it is not used
* `tools/mpy-tool.py`<br>changes needed for correct interpretation of floats for `MICROPY_OBJ_REPR_C` for RISC-V 64bit processor
* `lib/utils/pyexec.c`<br>changed printing of version info string
* small changes in all files dealing with PyStack<br>`MICROPY_ENABLE_PYSTACK` conditional compiles was changed to runtime check for PyStack enabled.<br>If PyStack is used or not can be changed in MicroPython configuration.

*All modified files contains the copyright string* **"Copyright (c) 2019 LoBo (https://github.com/loboris)"** *and the changes are marked with the* **LoBo** *comment.*<br>
All modifications are easy to maintain and should not present a problem when the MicroPython core is updated.

> *All files not used or needed for building the K210 Micropython port are removed.*

***

Latest update: May 29. 2019.<br>
Release: **1.11**<br>
SHA: `6f75c4f` (`6f75c4f3cd393131579db70cdf0b35d1fe5b95ab`)

***

[MicroPython **Release information**](https://github.com/micropython/micropython/releases/tag/v1.11)

[MicroPython **README.md**](https://github.com/micropython/micropython/blob/master/README.md)

***
