# Weather7573
## Requirements
* [TimerOne](https://github.com/PaulStoffregen/TimerOne)
* [DHT](https://github.com/adafruit/DHT-sensor-library)
* [Adafruit_BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)
* [TM1637Display](https://github.com/avishorp/TM1637)
* [RtcDS1302](https://github.com/Makuna/Rtc/blob/master/src/RtcDS1302.h)

## Wiring
* DHT 
 * out --> D5
 * '+' --> 5V
 * '-' --> GND
* BMP280
 * VCC --> 5V
 * GND --> GND
 * SCL --> A4
 * SDA --> A5
* TM1637 
 * CLK --> 12
 * DIO --> 13
* RtcDS1302
 * DAT --> 7
 * CLK --> 6
 * RST --> 8