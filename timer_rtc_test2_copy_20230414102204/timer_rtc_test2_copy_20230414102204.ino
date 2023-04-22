
// CONNECTIONS:
// DS1302 CLK/SCLK --> 5
// DS1302 DAT/IO --> 4
// DS1302 RST/CE --> 2
// DS1302 VCC --> 3.3v - 5v
// DS1302 GND --> GND

#include <ThreeWire.h>  
#include <RtcDS1302.h>
#include <TimerOne.h>

ThreeWire myWire(4,5,2); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

volatile unsigned int  timerCount2; 
const int PRESSURES_SIZE = 25;
float pressures[PRESSURES_SIZE];
int p = 0;
boolean fired = false;
boolean fired_hourly = false;
boolean fired_daily = false;
int s = 0;

void setup () 
{
    Serial.begin(57600);
    Serial.println("timer RTC test");
    Rtc.Begin();
    RtcDateTime now = Rtc.GetDateTime();
    printDateTime(now);
    Serial.println();

    // Setup the timer intterupt
    // stündlich --> minütlich polling freq
    Timer1.initialize(500000); // 0.5 second 500ms (minütlich)
    //Timer1.initialize(3000); // 0.5 min 30s (reicht für stündlich, geht aber nicht, weil anderes interrupt braucht das so)
    Timer1.attachInterrupt(timeReader);
    //Timer1.attachInterrupt(computeTendency);
    timerCount2 = 0;

  // fill pressures with values
  //prefillPressures();
}

void loop () 
{
  //Serial.println("doing something");
  String command = Serial.readString();
  if(command == "computeTendency\n") {
    printPressures();
    computeTendency();
  }

  /*runtimeTest();
  runtimeTest();
  runtimeTest();
  runtimeTest();
  while(1);*/
}

void runtimeTest() {
  Serial.println("runtime test");
  long starttime = millis();
  long duration;
  int iterations = 30000;
  for(int i = 0; i < iterations; i++)
    timeReader();
  duration = millis() - starttime;
  Serial.print("Laufzeit: ");
  Serial.print(duration);
  Serial.print(" ms ");
  Serial.print(iterations);
  Serial.print(" iterations, ");
  Serial.print((float)duration / (float) iterations);
  Serial.println(" ms/iteration");

}

void prefillPressures() {
  //int entry = 1400;
  float entry = (float) random(0, 10000) / 100.0;
  for(int i = 0; i < PRESSURES_SIZE; i++) {
    pressures[i] = (float) entry;
    //entry += 100;
    //if(entry > 2300) entry = 0;
    entry = (float) random(0, 10000) / 100.0;       
  }
}

void printPressures() {
    for (int i = 0; i < PRESSURES_SIZE; i++) {
    Serial.print(pressures[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void computeTendency() {
  int p0 = p - 1;
  int p1 = p;
  if(p0 < 0) p0 = PRESSURES_SIZE-1;
  /*Serial.print("p0: ");
  Serial.print(p0);
  Serial.print(", p1: ");
  Serial.print(p1);*/
  //Serial.print(", pressures[p0]: ");
  Serial.print(pressures[p0]);
  Serial.print(" ");
  //Serial.print(", pressures[p1]: ");
  Serial.print(pressures[p1]);
  Serial.print(" ");
  float tendency;
  float pressure0 = round(pressures[p0]*10.0)/10.0;
  float pressure1 = round(pressures[p1]*10.0)/10.0;
  Serial.print(pressure0);
  Serial.print(" ");
  Serial.print(pressure1);
  if(pressure0 >= pressure1)
    tendency = pressure0 - pressure1; // steigend
  else
    tendency = -1 * (pressure1 - pressure0); // fallend
  Serial.print(", tendency: ");
  Serial.println(tendency);
  String tendency_str;
  if(tendency > 0) tendency_str = "rising";
  else if(tendency == 0) tendency_str = "steady";
  else tendency_str = "falling";
  Serial.println(tendency_str);
}

void storePressure(RtcDateTime now) {
  //pressures[p] = ((float) random()) / 100.0;
  pressures[p] = (float) now.Hour()*100.0+now.Minute()+now.Second()/100.0;
  p++;
  if (p >= PRESSURES_SIZE) p = 0;
}

void timeReader() {
  timerCount2++;
  if(timerCount2 = 60) {  
    //Serial.println("timereader interrupt");
    RtcDateTime now = Rtc.GetDateTime();

    //printDateTime(now);
    //Serial.println();

    if (!now.IsValid())
    {
        // Common Causes:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }

    if((now.Second() > s)) {
      s = now.Second(); if(s==59) s = -1;
      //storePressure(now);
      //printPressures();
    }

    // every minute
    if((now.Second() == 0) && !fired) { 
      //storePressure(now);
      //printPressures();
      fired = true;
    }
    if((now.Second() == 1) && fired) { 
      //Serial.println("every minute reset");
      fired = false;
    }    
    // every hour
    if((now.Minute() == 0) && (now.Second() == 0) && !fired_hourly) { 
      Serial.println("every hour");
      storePressure(now);
      printPressures();
      fired_hourly = true;
    }
    if((now.Minute() == 0)  && (now.Second() == 1) && fired_hourly) { 
      Serial.println("every hour reset");
      fired_hourly = false;
    }

    if((now.Hour() == 0) && (now.Second() == 0) && !fired_daily) { 
      Serial.println("every day");
      fired_daily = true;
    }
    if((now.Hour() == 0) && (now.Second() == 1) && fired_daily) { 
      Serial.println("every day reset");
      fired_daily = false;
    }
    timerCount2 = 0;
  } // timerCount2
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