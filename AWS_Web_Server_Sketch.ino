#include "TimerOne.h"  // Timer Interrupt set to 2 second for read sensors

#include "DHT.h"
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

#define DHTPIN 5  // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define WindSensor_Pin (2)  // The pin location of the anemometer sensor
#define WindVane_Pin (A2)   // The pin the wind vane sensor is connected to
#define VaneOffset 0        // define the anemometer offset from magnetic north

volatile unsigned int timerCount;          // used to determine 2.5sec timer count
volatile unsigned long rotations;          // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in interrupt routine

volatile int minute = 0;
volatile int hour = 0;
int minute_offset = 0;
int hour_offset = 0;

volatile float windspeed;
int vaneValue;      // raw analog value from wind vane
int vaneDirection;  // translated 0 - 360 direction
int calDirection;   // converted value with offset applied

int winddir;
float temperature;
float pressure;
String tendency_str;

const int PRESSURES_SIZE = 25;
float pressures[PRESSURES_SIZE];
int p = 0;
boolean fired = false;
boolean fired_hourly = false;
//int c = 0; // count time units for printPressure()

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // setup anemometer values
  rotations = 0;
  // setup timer values
  timerCount = 0;

  pinMode(WindSensor_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

  // Setup the timer intterupt for 0.5 second
  Timer1.initialize(500000);
  Timer1.attachInterrupt(isr_timer);
  sei();  // Enable Interrupts

  dht.begin();
  Serial.begin(9600);

  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  //delay(1000);
  //int addr = 0x76;
  //status = bmp.begin(0x77);
  for (int addr = 0x76; addr < 0x100; addr++) {
    status = bmp.begin(addr);
    if (!status) {
      //Serial.println(addr, HEX);
    } else {
      Serial.println(addr, HEX);
      if (bmp.sensorID() > 0) break;
    }
  }
  Serial.print("bmp sensorID: ");
  Serial.println(bmp.sensorID());

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //bmp_temp->printSensorDetails();

  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  pressure = pressure_event.pressure + 49;
  if(isnan(pressure) || pressure < 955 || pressure > 1075) pressure = 1000.0;
  initPressuresWithArrayData(pressure);
  computeTendency();
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  // read sensors
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  temperature = dht.readTemperature();
  pressure = pressure_event.pressure + 49;
  //windspeed
  winddir = getwinddir();
  
  // fake values
  /*temperature = random(-10,50);
  pressure = random(960, 1070);
  windspeed = random(0, 180);
  winddir = random(0, 360);*/

  // write validated sensor data to serial
  digitalWrite(LED_BUILTIN, HIGH);
  computeMinute();
  /*Serial.print(hour);
  Serial.print(":");
  Serial.print(minute);
  Serial.print(":");
  Serial.print(millis());
  Serial.print(" ");*/
  if (isnan(temperature) || temperature < -30 || temperature > 100) {
    Serial.print("X");
  } else {
    Serial.print(temperature, 1);
    Serial.print("o");
    Serial.print("C");
  }
  Serial.print(" ");
  if (isnan(pressure) || pressure < 955 || pressure > 1075) {
    bmp.reset();
    Serial.print("X");
  } else {
    Serial.print(pressure, 1);
    Serial.print("hPa");
  }
  Serial.print(" ");
  Serial.print(tendency_str);
  Serial.print(" ");
  if (isnan(windspeed) || windspeed < 0 || windspeed > 200) {
    Serial.print("X");
  } else {
    Serial.print(windspeed, 1);
    Serial.print("km/h");
  }
  Serial.print(" ");
  if (isnan(winddir) || winddir < 0) {
    Serial.print("X");
  } else {
    Serial.print(convertWind(winddir + 180));
  }
  Serial.println();

  //if(c == 10) { printPressures(); c = 0; }
  //c++;
  digitalWrite(LED_BUILTIN, LOW);

  delay(200);
}

void resetBMP() {
  unsigned status;
  bmp.reset();
  for (int addr = 0x76; addr < 0x100; addr++) {
    status = bmp.begin(addr);
    if (!status) {
      //Serial.println(addr, HEX);
    } else {
      Serial.println(addr, HEX);
      if (bmp.sensorID() > 0) break;
    }
  }
  Serial.print("bmp sensorID: ");
  Serial.println(bmp.sensorID());

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //bmp_temp->printSensorDetails();
}

String convertWind(int degree) {
  degree = degree + 22;
  degree = degree % 360;
  //Serial.println(degree);
  if (degree < 45) {
    return "N";
  } else if (degree < 90) {
    return "NO";
  } else if (degree < 135) {
    return "O";
  } else if (degree < 180) {
    return "SO";
  } else if (degree < 225) {
    return "S";
  } else if (degree < 270) {
    return "SW";
  } else if (degree < 315) {
    return "W";
  } else if (degree < 360) {
    return "NW";
  } else {
    return "invalid";
  }
}

// isr routine for timer interrupt
void isr_timer() {
  timerCount++;
  if (timerCount % 5 == 0) {
    windspeed = rotations * 0.9 * 1.609 * 0.09;
    rotations = 0;
  }
  if (timerCount == 60) {
    if ((minute == 0) && !fired_hourly) {
      if(isnan(pressure)) storePressure(1100.0);
      else storePressure(pressure);
      //printPressures();
      computeTendency();
      fired_hourly = true;
    }
    if ((minute == 1) && fired_hourly) {
      fired_hourly = false;
    }
    timerCount = 0;
  }
}

// interrupt handler to increment the rotation count for wind speed
void isr_rotation() {
  if ((millis() - contactBounceTime) > 15) {  // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }
}

// Get Wind Direction
int getwinddir() {
  vaneValue = analogRead(WindVane_Pin);
  vaneDirection = map(vaneValue, 0, 1023, 0, 360);
  calDirection = vaneDirection + VaneOffset;

  if (calDirection > 360)
    calDirection = calDirection - 360;
  if (calDirection < 0)
    calDirection = calDirection + 360;
  return calDirection;
}

float correction[] = { 0, 0.518, 1.0, 1.414, 1.732, 1.932, 2.0, 1.932, 1.732, 1.414, 1.0, 0.518, 0, -0.518, -1.0, -1.414, -1.732, -1.932, -2.0, -1.932, -1.732, -1.414, -1, -0.518, 0 };

// init pressure array with estimated pressures based on current pressure
void initPressuresWithArrayData(float pressure) {
  for (int p = 0; p < PRESSURES_SIZE; p++) {
    float value = pressure + correction[(getHour() + p) % 24];
    //Serial.println(value);
    pressures[p] = value;
  }
}

void storePressure(float pressure) {
Serial.print("store pressure ");
  Serial.println(pressure);
  pressures[p] = pressure;
  p++;
  if (p >= PRESSURES_SIZE) p = 0;
}

void printPressures() {
  for (int i = 0; i < PRESSURES_SIZE; i++) {
    Serial.print(pressures[i]);
    Serial.print(" ");
  }
  Serial.println();
}

String computeTendency() {
  int p0 = p - 1;
  int p1 = p;
  if (p0 < 0) p0 = PRESSURES_SIZE - 1;

  float tendency;
  float pressure0 = round(pressures[p0] * 10.0) / 10.0;
  float pressure1 = round(pressures[p1] * 10.0) / 10.0;
  if(isnan(pressure0) || pressure0 < 955 || pressure0 > 1075 || 
     isnan(pressure1) || pressure1 < 955 || pressure1 > 1075) {
    tendency_str = "invalid";
    return;
  }


  if (pressure0 >= pressure1)
    tendency = pressure0 - pressure1;  // rising
  else
    tendency = -1 * (pressure1 - pressure0);  // falling

  if (tendency > 0) tendency_str = "rising";
  else if (tendency == 0) tendency_str = "steady";  // steady
  else tendency_str = "falling";
  return tendency_str;
}

int computeMinute() {
  minute = (millis() / 1000 / 60) % 60;
}

int getHour() {
  computeMinute();
  hour = (minute / 60) % 24;
  return hour;
}