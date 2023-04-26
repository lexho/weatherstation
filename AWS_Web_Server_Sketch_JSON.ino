#include <SoftwareSerial.h>
SoftwareSerial softSerial(10, 11);

#include "TimerOne.h"          // Timer Interrupt set to 2 second for read sensors
#include <math.h>

#include "DHT.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// CONNECTIONS:
// DS1302 CLK/SCLK --> 5 6
// DS1302 DAT/IO --> 4 7
// DS1302 RST/CE --> 2 8
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND

#include <ThreeWire.h>  
#include <RtcDS1302.h>
ThreeWire myWire(7,6,8); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

#define DHTPIN 5  // Digital pin connected to the DHT sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22  // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 3 (on the right) of the sensor to GROUND (if your sensor has 3 pins)
// Connect pin 4 (on the right) of the sensor to GROUND and leave the pin 3 EMPTY (if your sensor has 4 pins)
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);

#define Bucket_Size 0.01   // bucket size to trigger tip count
#define TX_Pin 8           // used to indicate web data tx

#define WindSensor_Pin (2)      // The pin location of the anemometer sensor
#define WindVane_Pin (A2)       // The pin the wind vane sensor is connected to
#define VaneOffset 0        // define the anemometer offset from magnetic north

//volatile unsigned long tipCount;    // bucket tip counter used in interrupt routine 
//volatile unsigned long contactTime; // Timer to manage any contact bounce in interrupt routine 

volatile unsigned int  timerCount;    // used to determine 2.5sec timer count
volatile unsigned long rotations;     // cup rotation counter used in interrupt routine
volatile unsigned long contactBounceTime;  // Timer to avoid contact bounce in interrupt routine

//long lastTipcount;            // keeps track of bucket tips 

volatile float windSpeed;
//float windSpeedMax; // replaced by function getWindSpeedMax()
int vaneValue;       // raw analog value from wind vane
int vaneDirection;   // translated 0 - 360 direction
int calDirection;    // converted value with offset applied
int lastDirValue;    // last recorded direction value 

float minTemp;   // keeps track of the minimum recorded temp value
float maxTemp;   // keeps track of the maximum recorded temp value  

// arayas we use for averaging sensor data
float temps[12];    // array of 12 temps to create a 2 minute average temp
float wspeeds[24];  // array of 24 wind speeds for 2 minute average wind speed
float wdirect[24];  // array of 24 wind directions for 2 minute average
int isp = 0; // Iterator for wspeeds-Array
int idr = 0; // Iterator for wdirect-Array

const int numValues = 4;
   float windSpeedMin;
   float windSpeedMax;
   int windDirection;
   float humidity;
   float temperature;
   float pressure;
   String tendency_str;
   //boolean p_fired = false;
   //boolean bmp_reset_fired = false;

volatile unsigned int  timerCount2; 
const int PRESSURES_SIZE = 25;
float pressures[PRESSURES_SIZE];
int p = 0;
boolean fired = false;
boolean fired_hourly = false;
boolean fired_daily = false;
int s = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); 
  digitalWrite(LED_BUILTIN, HIGH);
  Rtc.Begin();
  // setup anemometer values
  lastDirValue = 0;
  rotations = 0;
  //windSpeedMax = 0.0f; // replaced by function getWindSpeedMax()

  // setup timer values
  timerCount = 0;

  // disable the SD card by switching pin 4 high
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  //pinMode(RG11_Pin, INPUT); 
  pinMode(WindSensor_Pin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(WindSensor_Pin), isr_rotation, FALLING);

  // Setup the timer intterupt for 0.5 second
  Timer1.initialize(500000);
  Timer1.attachInterrupt(isr_timer);
  
  sei();// Enable Interrupts   

  dht.begin();
  Serial.begin(9600);
  softSerial.begin(9600); //115299

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
      if(bmp.sensorID() > 0) break;
    }
  }
  Serial.print("bmp.sensorID()");
  Serial.println(bmp.sensorID());

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //bmp_temp->printSensorDetails();

  sensors_event_t pressure_event;
  bmp_pressure->getEvent(&pressure_event);
  pressure = pressure_event.pressure + 49;
  initPressuresWithArrayData(pressure);
  computeTendency();
  digitalWrite(LED_BUILTIN, LOW);
}
  
void loop() { 
   // read sensors
  //windSpeedMin = getWindSpeedMin();
  //windSpeedMax = getWindSpeedMax();
  windDirection = getWindDirection();
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  sensors_event_t temp_event, pressure_event;

  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  pressure = pressure_event.pressure + 49;

  //Serial.print(temp_event.temperature);
  //Serial.println(" *C");
  //Serial.print(",");
  //Serial.print("Pressure:");
  //Serial.print(pressure_event.pressure);

    // Check if any reads failed and exit early (to try again).
  if (isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  /*Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("% Temperature: "));
  Serial.print(temperature);
  Serial.println(F("°C "));
  Serial.print(" windSpeed: ");
  Serial.print(windSpeed);
  Serial.print(" windSpeedMin: ");
  Serial.print(windSpeedMin);
  Serial.print(" windSpeedMax: ");
  Serial.print(windSpeedMax);
  Serial.print(" windDirection: ");
  Serial.println(windDirection);*/

/*//Serial.println((millis() / 1000) % 5);
  if((millis() / 1000) % 5 == 1) fired = false;
 if(!fired && (millis() / 1000) % 5 == 0){
    fired = true;
printJSON();
}*/

/*Serial.print(temperature);
Serial.print(" ");
Serial.print(-1);
Serial.print(" ");
Serial.print(windSpeed);
Serial.print(" ");
Serial.println(windDirection);

Serial.println(softSerial.readString());*/


//Serial.print(temperature);
//Serial.print(round(temperature, 1));
Serial.print(temperature,1);
Serial.print("o");
Serial.print("C ");
Serial.print(pressure,1);
Serial.print("hPa ");
//printPressures();
if(tendency_str.length() > 0) { Serial.print(tendency_str); Serial.print(" "); }
//Serial.print(windSpeed);
//Serial.print(round(windSpeed*10.0)/10.0);
Serial.print(windSpeed,1);
Serial.print("km/h ");
Serial.println(convertWind(windDirection+180));

digitalWrite(LED_BUILTIN, HIGH);
softSerial.print(temperature,1);
softSerial.print("o");
softSerial.print("C ");
softSerial.print(pressure,1);
softSerial.print("hPa ");
if(tendency_str.length() > 0) { softSerial.print(tendency_str); softSerial.print(" "); }
softSerial.print(windSpeed,1);
softSerial.print("km/h ");
softSerial.println(convertWind(windDirection+180));
digitalWrite(LED_BUILTIN, LOW);
delay(2000);
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
      if(bmp.sensorID() > 0) break;
    }
  }
  Serial.print("bmp.sensorID()");
  Serial.println(bmp.sensorID());

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
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

String printZehntel(int zehntel) {
  String output = "";
  if (zehntel < 0) {
    output.concat('-');
    zehntel = -zehntel;
  }
  output.concat(zehntel / 10);
  output.concat('.');
  output.concat(zehntel % 10);
  return output;
}
// isr routine for timer interrupt
void isr_timer() {
  
  timerCount++;
  
  if(timerCount % 5 == 0)
  //if(timerCount == 5)
  {
    windSpeed = rotations * 0.9 * 1.609 * 0.09;
    wspeeds[isp] = windSpeed;
    rotations = 0;
    //timerCount = 0;
    if(isp >= 23) isp = 0; else isp++;
  }
  if(timerCount == 60) {  
    RtcDateTime now = Rtc.GetDateTime();

    printDateTime(now);
    Serial.println();

    if (!now.IsValid())
      Serial.println("RTC lost confidence in the DateTime!");
  
    // every hour
    if((now.Minute() == 0) && !fired_hourly) { 
      storePressure(pressure);
      printPressures();
      computeTendency();
      fired_hourly = true;
    }
    if((now.Minute() == 1)  && fired_hourly) { 
      fired_hourly = false;
    }
    timerCount = 0;
  }
}

// interrupt handler to increment the rotation count for wind speed
void isr_rotation ()   {

  if ((millis() - contactBounceTime) > 15 ) {  // debounce the switch contact.
    rotations++;
    contactBounceTime = millis();
  }
}

// Get Wind Direction
int getWindDirection() {
 
  //vaneValue = analogRead(WindVane_Pin);
  wdirect[idr] = analogRead(WindVane_Pin);
  if(idr >= 23) idr = 0; else idr++;

  // compute average vaneValue
  int sum = 0;
  for(int i = 0; i < 24; i++) {
    sum += wdirect[i];
  }
  vaneValue = sum / 24;
   
   //calDirection = vaneValue;
   vaneDirection = map(vaneValue, 0, 1023, 0, 360);
   //vaneDirection = map(vaneValue, 0, 422, 0, 360);
   calDirection = vaneDirection + VaneOffset;
   
   if(calDirection > 360)
     calDirection = calDirection - 360;
     
   if(calDirection < 0)
     calDirection = calDirection + 360;
  return calDirection;
}

float getWindSpeed() {
return windSpeed;  
}

float getWindSpeedMax() {
    float speed_max = 0.0;
    for(int i = 0; i < 24; i++) {
      if(wspeeds[i] > speed_max) speed_max = wspeeds[i];
    }
    return speed_max;
}

float getWindSpeedMin() {
    float speed_min = wspeeds[0];
    for(int i = 0; i < 24; i++) {
      if(wspeeds[i] < speed_min) speed_min = wspeeds[i];
    }
    return speed_min;
}
/*void resetWindSpeedMax() {
  windSpeedMax = 0.0f;  
}*/

// init pressure array with current pressure
void initPressures(float pressure) {
  for(int p = 0; p < PRESSURES_SIZE; p++) {
    pressures[p] = pressure;
  }
}

float korrektur[] = { 0, 0.518, 1.0, 1.414, 1.732, 1.932, 2.0, 1.932, 1.732, 1.414, 1.0, 0.518, 0, -0.518, -1.0, -1.414, -1.732, -1.932, -2.0, -1.932, -1.732, -1.414, -1, -0.518, 0 };

// init pressure array with estimated pressures based on current pressure
void initPressuresWithArrayData(float pressure) {
  RtcDateTime now = Rtc.GetDateTime();
  for(int p = 0; p < PRESSURES_SIZE; p++) {
    pressures[p] = pressure + korrektur[(now.Hour()+p) % 24];
  }
}

float sinewave(int x) {
  return sin(0.2618*x)*(4/2);
}
void initPressuresWithFormular(float pressure) {
  RtcDateTime now = Rtc.GetDateTime();
  for(int p = 0; p < PRESSURES_SIZE; p++) {
    pressures[p] = pressure + sinewave((now.Hour()+p) % 24);
  }
}

void storePressure(float pressure) {
  //pressures[p] = ((float) random()) / 100.0;
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
  if(p0 < 0) p0 = PRESSURES_SIZE-1;
  /*Serial.print("p0: ");
  Serial.print(p0);
  Serial.print(", p1: ");
  Serial.print(p1);*/
  //Serial.print(", pressures[p0]: ");
  //Serial.print(pressures[p0]);
  //Serial.print(" ");
  //Serial.print(", pressures[p1]: ");
  //Serial.print(pressures[p1]);
  //Serial.print(" ");
  float tendency;
  float pressure0 = round(pressures[p0]*10.0)/10.0;
  float pressure1 = round(pressures[p1]*10.0)/10.0;
  //Serial.print(pressure0);
  //Serial.print(" ");
  //Serial.print(pressure1);
  if(pressure0 >= pressure1)
    tendency = pressure0 - pressure1; // steigend
  else
    tendency = -1 * (pressure1 - pressure0); // fallend
  //Serial.print(", tendency: ");
  //Serial.println(tendency);
  if(tendency > 0) tendency_str = "rising";
  else if(tendency == 0) tendency_str = "steady";
  else tendency_str = "falling";
  //Serial.println(tendency_str);
  return tendency_str;
}

void printJSON() { 
               Serial.println("{");
             Serial.print("\t\"wind\": {"); Serial.println();    
                Serial.print("\t\t\"speed\": {"); Serial.println();    
                    Serial.print("\t\t\t\"value\": "); Serial.print(windSpeed); Serial.print(","); Serial.println(); 
                    Serial.print("\t\t\t\"min\": "); Serial.print(windSpeedMin); Serial.print(","); Serial.println();
                    Serial.print("\t\t\t\"max\": "); Serial.print(windSpeedMax); Serial.print(","); Serial.println();  
                    Serial.print("\t\t\t\"unit\": "); Serial.print("\"km/h\""); Serial.println(); 
                Serial.print("\t\t},"); Serial.println();
                Serial.print("\t\t\"direction\": {"); Serial.println();
                    Serial.print("\t\t\t\"value\": "); Serial.print(windDirection); Serial.print(","); Serial.println();
                    Serial.print("\t\t\t\"unit\": "); Serial.print("\"degree\""); Serial.println(); 
                Serial.print("\t\t}"); Serial.println();
             Serial.print("\t},"); Serial.println();
            Serial.println("\t\"temperature\": {");
                Serial.print("\t\t\"value\": "); Serial.print(temperature); Serial.println(","); 
                Serial.print("\t\t\"unit\": "); Serial.println("\"°C\"");
             Serial.println("\t}");
             Serial.println("}");
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}
