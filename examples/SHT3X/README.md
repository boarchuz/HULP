An example of using i2c bit-banging to read an sht3x sensor every 10s but only waking up the ESP32 when temperature has changed by 0.1°C.
Also includes some settings in `sdkconfig.defaults` to reduce deep sleep wakeup time, allowing wakeup in 41ms.

Example output:
```
rst:0x5 (DEEPSLEEP_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0030,len:1412
load:0x40078000,len:13172
load:0x40080400,len:3608
0x40080400: _init at ??:?

entry 0x400805f0
I (41) MAIN: temp: 19.8, last_temp: 19.9, difference: -0.11, hum: 54.6
I (41) MAIN: Sleeping
ets Jul 29 2019 12:21:46

rst:0x5 (DEEPSLEEP_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
configsip: 0, SPIWP:0xee
clk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00
mode:DIO, clock div:1
load:0x3fff0030,len:1412
load:0x40078000,len:13172
load:0x40080400,len:3608
0x40080400: _init at ??:?

entry 0x400805f0
I (41) MAIN: temp: 19.7, last_temp: 19.8, difference: -0.11, hum: 55.3
I (41) MAIN: Sleeping
```
