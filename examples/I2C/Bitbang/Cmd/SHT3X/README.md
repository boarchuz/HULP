An example of using i2c bit-banging to read an sht3x sensor every 10s but only waking up the ESP32 when temperature has changed by 0.1°C.

Example output:
```
rst:0x5 (DEEPSLEEP_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
...
I (337) MAIN: temp: 19.8, last_temp: 19.9, difference: -0.11, hum: 54.6
I (347) MAIN: Sleeping

rst:0x5 (DEEPSLEEP_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
...
I (337) MAIN: temp: 19.7, last_temp: 19.8, difference: -0.11, hum: 55.3
I (347) MAIN: Sleeping
```
