
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

float pressures[24];
int p = 0;
boolean fired = false;
boolean fired_hourly = false;
boolean fired_daily = false;

void setup () 
{
    Serial.begin(57600);
    Serial.println("timer RTC test");
    Rtc.Begin();
    RtcDateTime now = Rtc.GetDateTime();
    printDateTime(now);
    // Setup the timer intterupt for 0.5 second
    Timer1.initialize(500000);
    Timer1.attachInterrupt(timeReader);
}

void timeReader() {
RtcDateTime now = Rtc.GetDateTime();

    //printDateTime(now);
    //Serial.println();

    if (!now.IsValid())
    {
        // Common Causes:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
    }
    // every minute
    //Serial.println(now.Second());
    //Serial.println(now.Minute());
    if((now.Second() == 0) && !fired) { 
      //Serial.println("every minute");
      //pressures[p] = ((float) random()) / 100.0;
      /*pressures[p] = (float) now.Hour()*100.0+now.Minute()+now.Second()/100.0;
      p++;
      if (p > 23) p = 0;
      for (int i = 0; i < 24; i++) {
        Serial.print(pressures[i]);
        Serial.print(" ");
      }
      Serial.println();*/
      fired = true;
    }
    if((now.Second() == 1) && fired) { 
      //Serial.println("every minute reset");
      fired = false;
    }
    // every hour
    if((now.Minute() == 0) && !fired_hourly) { 
      Serial.println("every hour");
      //pressures[p] = ((float) random()) / 100.0;
      pressures[p] = (float) now.Hour()*100.0+now.Minute()+now.Second()/100.0;
      p++;
      if (p > 23) p = 0;
      for (int i = 0; i < 24; i++) {
        Serial.print(pressures[i]);
        Serial.print(" ");
      }
      Serial.println();
      fired_hourly = true;
    }
    if((now.Minute() == 1) && fired_hourly) { 
      Serial.println("every hour reset");
      fired_hourly = false;
    }

    if((now.Hour() == 0) && !fired_daily) { 
      Serial.println("every day");
      fired = true;
    }
    if((now.Hour() == 1) && fired_daily) { 
      Serial.println("every day reset");
      fired = false;
    }
}

void loop () 
{
  
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

