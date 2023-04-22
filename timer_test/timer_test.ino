float pressures[24];
int p = 0;
boolean p_fired = false;
boolean bmp_reset_fired = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("testing timer");
}

void loop() {
  if (((millis() / 1000) % 60 * 60 * 24 == 0) && !bmp_reset_fired) { // every day (24h)
    Serial.print(millis());
    Serial.println("ms");
    Serial.println("bmp.reset();");  // sometimes we need a reset (every 24h)
    bmp_reset_fired = true;
  }
  if ((millis() / 1000) % 60 * 60 * 24 == 1 && bmp_reset_fired) {
    Serial.print(millis());
    Serial.println("ms");
    Serial.println("bmp_reset() reset");
    bmp_reset_fired = false;
  }
  if (((millis() / 1000) % 60 * 60 == 0) && !p_fired) {  // every hour
    Serial.print(millis());
    Serial.println("ms");
    pressures[p] = ((float) random()) / 100.0;
    p++;
    if (p > 23) p = 0;
    p_fired = true;
     Serial.print("pressures[] ");
    for (int i = 0; i < 24; i++) {
      Serial.print(pressures[i]);
      Serial.print(" ");
    }
          Serial.println();
  }

  if (((millis() / 1000) % 60 * 60 == 1) && p_fired) {
    Serial.println("p reset");
    p_fired = false;
  }

  delay(500);
}
