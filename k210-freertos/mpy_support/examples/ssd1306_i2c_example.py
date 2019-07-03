import machine
from ssd1306 import SSD1306_I2C
from ssd1306 import SH1106_I2C

WIDTH = 128
HEIGHT = 64
sda_pin = 20
scl_pin = 21

i2c = machine.I2C(0, scl=scl_pin, sda=sda_pin, speed=400000)

ssd = SSD1306_I2C(WIDTH, HEIGHT, i2c)
#ssd = SH1106_I2C(WIDTH, HEIGHT, i2c)

ssd.fill(0)
ssd.rect(0,0,WIDTH,HEIGHT,1)
ssd.text("MicroPython",3,3)
ssd.show()

import freesans20

from writer import Writer
wri2 = Writer(ssd, freesans20, verbose=True)

Writer.set_clip(True, True)
Writer.set_textpos(0, 0)
wri2.printstring('MicroPython\nby LoBo\n10/2019')

ssd.show()
