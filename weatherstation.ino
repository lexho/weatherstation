/*
  Wetterstation - Arduino Uno
  Sensoren: Davis Windmesser + Davis Windfahne, DHT22, BMP280

  Bibliotheken (Bibliotheksverwalter):
    - "DHT sensor library" (Adafruit)
    - "Adafruit BMP280 Library" (zieht Unified Sensor + BusIO nach)

  Verdrahtung (übliche Davis-RJ11-Belegung, bitte mit eurem Kabel prüfen):
    Windmesser-Kontakt (schwarz) -> D2   (interner Pull-up, Kontakt schaltet gegen GND)
    GND (rot)                    -> GND
    Windfahne Schleifer (grün)   -> A2
    Windfahne Versorgung (gelb)  -> 5V
    DHT22 Daten                  -> D5
    BMP280 SDA/SCL               -> A4/A5
*/

#include <Wire.h>
#include <avr/wdt.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>

// ---------- Pins ----------
const uint8_t PIN_DHT        = 5;
const uint8_t PIN_WIND_SPEED = 2;   // INT0
const uint8_t PIN_WIND_VANE  = A2;

// ---------- Konfiguration ----------
const int16_t       VANE_OFFSET_DEG      = 180;        // nach Montage kalibrieren (z. B. 180)
const unsigned long WIND_INTERVAL_MS     = 2500;     // Messintervall Wind + Ausgabe
const unsigned long PRESSURE_INTERVAL_MS = 3600000UL; // Druck für Tendenz: 1x pro Stunde
const float         KMH_PER_HZ           = 3.621;    // Davis: 2.25 mph pro Hz = 3.621 km/h pro Hz
const float         TREND_THRESHOLD_HPA  = 1.0;      // Schwelle für "rising"/"falling" über 3 h
const uint8_t       DEBOUNCE_MS          = 15;

// ---------- Objekte ----------
DHT dht(PIN_DHT, DHT22);
Adafruit_BMP280 bmp;

// ---------- Wind (ISR) ----------
volatile uint8_t       rotations   = 0;
volatile unsigned long lastPulseMs = 0;

// ---------- Druckverlauf (Ringpuffer, 1 Wert pro Stunde) ----------
const uint8_t HISTORY_SIZE = 25;
float   pressureHistory[HISTORY_SIZE];
uint8_t historyHead  = 0;   // nächster Schreibindex
uint8_t historyCount = 0;

unsigned long lastWindMs;
unsigned long lastPressureStoreMs;

const char *const COMPASS[8] = { "N", "NO", "O", "SO", "S", "SW", "W", "NW" };

// ---------- ISR ----------
void isr_rotation() {
  unsigned long now = millis();
  if (now - lastPulseMs > DEBOUNCE_MS) {
    rotations++;
    lastPulseMs = now;
  }
}

// ---------- BMP280 ----------
bool initBMP() {
  //bool ok = bmp.begin(0x76) || bmp.begin(0x77);
  bool status = false;
  for (int addr = 0x76; addr < 0x100; addr++) {
    status = bmp.begin(addr);
    if (!status) {
      //Serial.println(addr, HEX);
    } else {
      Serial.println(addr, HEX);
      if (bmp.sensorID() > 0) break;
    }
  }
  if (status) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X1,
                    Adafruit_BMP280::SAMPLING_X8,
                    Adafruit_BMP280::FILTER_OFF,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
  return status;
}

// Stationsdruck in hPa (kein Meeresspiegeldruck!), NAN bei ungültigem Wert
float readPressureHpa() {
  float p = bmp.readPressure() / 100.0f;
  if (isnan(p) || p < 300.0f || p > 1100.0f) return NAN;
  return p;
}

// ---------- Druckverlauf ----------
void storePressure(float p) {
  pressureHistory[historyHead] = p;
  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// Änderung über 3 Stunden (aktueller Wert vs. Wert vor 3 h)
const char *computeTendency() {
  if (historyCount < 4) return "--";   // noch keine 3 h Daten
  float current = pressureHistory[(historyHead + HISTORY_SIZE - 1) % HISTORY_SIZE];
  float before  = pressureHistory[(historyHead + HISTORY_SIZE - 4) % HISTORY_SIZE];
  if (isnan(current) || isnan(before)) return "--";
  float diff = current - before;
  if (diff >  TREND_THRESHOLD_HPA) return "rising";
  if (diff < -TREND_THRESHOLD_HPA) return "falling";
  return "steady";
}

// ---------- Windfahne ----------
int readWindDirection() {
  int raw = analogRead(PIN_WIND_VANE);
  int deg = (int)((long)raw * 360 / 1023);
  deg = (deg + VANE_OFFSET_DEG) % 360;
  if (deg < 0) deg += 360;
  return deg;
}

const char *compassName(int deg) {
  return COMPASS[((deg * 2 + 45) / 90) % 8]; // 360 --> 0 - 8
}

// ---------- Ausgabe ----------
void printValue(float v, uint8_t decimals, const __FlashStringHelper *unit) {
  if (isnan(v)) {
    Serial.print(F("X"));
  } else {
    Serial.print(v, decimals);
    Serial.print(unit);
  }
}

void printUptime(unsigned long nowMs) {
  unsigned long s = nowMs / 1000UL;
  Serial.print(s / 3600UL);
  Serial.print(':');
  uint8_t m = (s / 60UL) % 60UL;
  if (m < 10) Serial.print('0');
  Serial.print(m);
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_WIND_SPEED, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_WIND_SPEED), isr_rotation, FALLING);

  Wire.begin();
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);   // I2C-Hänger abfangen
#endif

  dht.begin();
  if (!initBMP()) Serial.println(F("BMP280 nicht gefunden"));

  storePressure(readPressureHpa());

  lastWindMs = lastPressureStoreMs = millis();
  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();

  unsigned long now = millis();
  unsigned long elapsed = now - lastWindMs;
  if (elapsed < WIND_INTERVAL_MS) return;
  lastWindMs = now;

  // Wind
  noInterrupts();
  uint8_t count = rotations;
  rotations = 0;
  interrupts();
  float windKmh = (count * 1000.0f / elapsed) * KMH_PER_HZ;
  int dir = readWindDirection();

  // Temperatur / Feuchte
  float temperature = dht.readTemperature();
  float humidity    = dht.readHumidity();
  if (isnan(temperature) || temperature < -40 || temperature > 80) temperature = NAN;
  if (isnan(humidity) || humidity < 0 || humidity > 100) humidity = NAN;

  // Druck (bei ungültigem Wert BMP280 neu initialisieren)
  float pressure = readPressureHpa();
  if (isnan(pressure)) initBMP();

  // Stündlich für die Tendenz speichern
  if (now - lastPressureStoreMs >= PRESSURE_INTERVAL_MS) {
    lastPressureStoreMs += PRESSURE_INTERVAL_MS;
    storePressure(pressure);
  }

  // Ausgabe
  printUptime(now);
  Serial.print(' ');
  printValue(temperature, 1, F("oC"));
  Serial.print(' ');
  printValue(humidity, 0, F("%"));
  Serial.print(' ');
  printValue(pressure, 1, F("hPa"));
  Serial.print(' ');
  Serial.print(computeTendency());
  Serial.print(' ');
  printValue(windKmh, 1, F("km/h"));
  Serial.print(' ');
  Serial.print(compassName(dir));
  Serial.print(' ');
  Serial.print(dir);
  Serial.println(F("deg"));
}
