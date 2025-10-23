# Weatherstation
This weather station measures the following values: temperature, pressure, wind speed, wind direction. Every second it prints one line with current data on the console.

## Requirements
* [TimerOne](https://github.com/PaulStoffregen/TimerOne)
* [DHT](https://github.com/adafruit/DHT-sensor-library)
* [Adafruit_BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)

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

## Sample Output
```20.0 oC 1018.1hPa rising 16.0km/h W```
