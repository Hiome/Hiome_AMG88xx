# Hiome AMG88xx Library

This library was forked from [Adafruit](https://github.com/adafruit/Adafruit_AMG88xx), but has been significantly refactored for higher accuracy and speed while also using less memory. It is used by [Hiome Door](https://hiome.com) to interface with its AMG8833 thermopile sensor.

Major changes:
* Refactored logic based on [reference code from Panasonic](http://docplayer.net/54576058-C-source-code-for-general-use.html)
* Refactored to use [lightweight I2C library](https://github.com/rambo/I2C)
* I2C runs in fast mode
* Added power mode methods
* `readPixels` returns true if pixels changed since last read, and supports constraining returned values
