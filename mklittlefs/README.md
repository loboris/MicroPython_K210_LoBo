<br>

## Using prepared **LittleFS** image

**Prepared** image file can be flashed to K210 board, if not flashed, the file system will be formated after first boot.


LittleFS **image** can be **prepared** on host and flashed to K210 board:

---

### Prepare the image

Copy the files to be included on LittleFS into **`mklittlefs/internalfs_image/`** directory. Subdirectories can also be added.

Change the working directory to `mklittlefs`.<br>
The image file is created using `mklfs` utility.

```
Usage:
  mklfs [-b block_size] [-c block_count] [-l lookahead_size] image_dir image_name
      block_size: default=512   (MICRO_PY_LITTLEFS_SECTOR_SIZE)
     block_count: default=20480 (MICRO_PY_FLASHFS_SIZE / MICRO_PY_LITTLEFS_SECTOR_SIZE)
  lookahead_size: default=32    (LITTLEFS_CFG_LOOKAHEAD_SIZE)
```
If using the dafault Flash file system parameters defined in `mpconfigport.h`, you only have to provide input directory and output file name<br>


Execute:
```
./mklfs <image_input_dir> <image_name>
```

Example:
```
./mklfs internalfs_image maixpy_lfs.img

Creating LittleFS image
=======================
Image directory:
  'internalfs_image'
Image name:
  'maixpy_lfs.img'
Block size=512, Block count=20480, lookahead=32

Creating and mounting LittleFS image...
  Image file created.
  Image file formated.
  Image file mounted.

Adding files from image directory:
  'internalfs_image'
----------------------------------

/examples [D]
/examples/display [D]
/examples/display/test1.jpg
/examples/display/test2.jpg
/examples/display/test3.jpg
/examples/display/test4.jpg
/examples/display/tiger.bmp
/examples/webserver [D]
/examples/webserver/README.md
/examples/webserver/webserver_example.py
/examples/bme280.py
/examples/sqlite3_example.py
/examples/chinook.db
/examples/wifi_example.py
/examples/ssd1306_i2c_example.py
/fonts [D]
/fonts/arial_bold.fon
/fonts/BigFont.fon
/fonts/DejaVuSans12.fon
/fonts/DejaVuSans18.fon
/fonts/DejaVuSans24.fon
/fonts/DotMatrix_M.fon
/fonts/Grotesk24x48.fon
/fonts/OCR_A_Extended_M.c
/fonts/swiss721_outline.fon
/fonts/Georgia128.fon
/fonts/dot32.fon
/fonts/lobo30.fon
/lib [D]
/www [D]
/www/python.ico
/www/favicon.ico
/www/test.pyhtml
/www/wstest.html
/www/style.css
/www/index.html
/www/pdf.png
/www/pdf-sample.pdf
/boot.py

Image size: 1506897 (1507328)
Saving image to 'maixpy_lfs.img'
=======================

```

### Flash the image

Change the working directory to `k210-freertos`.

The image must be flashed to the file system start address defined in `mpconfigport.h` (`MICRO_PY_FLASHFS_START_ADDRESS`).<br>
Default address is at 4MB (4194304).

To flash it, execute:
```
./kflash.py -p /dev/ttyUSB0 -b 2000000 --address 4194304 -t ../mklittlefs/maixpy_lfs.img
```

Change */dev/ttyUSB0* to the port used to connect to the board if needed.

---

