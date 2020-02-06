
***

This is the clone of the main [MicroPython repository](https://github.com/micropython/micropython).<br><br>
It is mostly unchanged, except for the following files:<br>

* Renamed unused files to prevent compiling
  * `extmod/modlwip.c` --> `extmod/modlwip.c.orig`
* Completelly rewritten thread support
  * `py/mpthread.h`
  * `py/modthread.c`
* Changes needed for correct interpretation of floats for `MICROPY_OBJ_REPR_C` and `MICROPY_OBJ_REPR_D` for RISC-V 64bit processor
  * `py/obj.h`
* Changes needed for using `ll` for 64-bit integers (`MICROPY_OBJ_REPR_C`)
  * `py/mpprint.c`
* Small changes to enable support for two MicroPython instances running on two processors:
  * `py/runtime.c`
  * `py/builtin.h`
  * `py/gc.c`,
  * `py/gc.h`
  * `py/mpstate.h`
* Small changes to enable support for two MicroPython instances running on two processors<br>**`sys.path`, `sys.argv` _and_ `sys.modules` _are now FUNCTIONS:_ `sys.path()`, `sys.argv()` _and_ `sys.modules()`**
  * `py/modsys.c`
* Added PyStack info in `mem_info` function
  * `py/modmicropython.c`
* Small changes to address some errors when compiling without any native emmiter enabled
  * `py/compile.c`
  * `py/persistentcode.c`
* Added support for 64-bit memory objects
  * `extmod/machine_mem.c`
  * `extmod/machine_mem.h`
* Added some functions (circle, fill_circle)
  * `extmod/modframebuf.c`
* Changes needed for correct interpretation of floats for `MICROPY_OBJ_REPR_C` for RISC-V 64bit processor
  * `tools/mpy-tool.py`
* Changed printing of version info string
  * `lib/utils/pyexec.c`<br>
* Small changes in all files dealing with PyStack<br>`MICROPY_ENABLE_PYSTACK` conditional compiles was changed to runtime check for PyStack enabled.<br>If PyStack is used or not can be changed in MicroPython configuration.
  * `py/nlr.h`
  * `py/objbountmeth.c`
  * `py/objfun.c`
  * `py/qstrdefs.h`
  * `py/pystack.c`
  * `py/pystack.h`
  * `py/vm.c`
* Some changes in `mk` files for better integration with this port's build system
  * `py/mkrules.mk`
  * `py/mkenv.mk`
* Small change to avoid compiler errors when building on OSX
  * `py/mpstate.c`

*All modified files contains the copyright string* **"Copyright (c) 2019 LoBo (https://github.com/loboris)"** *and the changes are marked with the* **LoBo** *comment.*<br>
All modifications are easy to maintain and should not present a problem when the MicroPython core is updated.

> *All files not used or needed for building the K210 Micropython port are removed.*

***

Latest update: Dec 22. 2019.<br>
Release: **1.12**<br>
SHA: `1f37194` (`1f371947309c5ea6023b6d9065415697cbc75578`)

***

[MicroPython **Release information**](https://github.com/micropython/micropython/releases/tag/v1.12)

[MicroPython **README.md**](https://github.com/micropython/micropython/blob/master/README.md)

***
