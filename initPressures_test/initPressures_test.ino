const int PRESSURES_SIZE = 25;
float pressures[PRESSURES_SIZE];
int p = 0;
int hour = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  initPressuresBetterWithFormular(1020.0);
  printPressures();
  
  initPressuresBetter(1020.0);
  printPressures();
  
  while(1);
}

//wrong float korrektur[] = { 10.0, 8.33, 6.67, 5.0, 3.34, 1.67, 0.0, -1.67, -3.34, -5.0, -6.67, -8.33, -10.0, -8.33, -6.67, -5.0, -3.34, -1.67, 0.0, +1.67, +3.34, +5.0, +6.67, +8.33, +10.0 };
float korrektur[] = { 0, 0.518, 1.0, 1.414, 1.732, 1.932, 2.0, 1.932, 1.732, 1.414, 1.0, 0.518, 0, -0.518, -1.0, -1.414, -1.732, -1.932, -2.0, -1.932, -1.732, -1.414, -1, -0.518, 0 };
void initPressuresBetter(float pressure) {
  for(int p = 0; p < PRESSURES_SIZE; p++) {
    pressures[p] = pressure + korrektur[(hour+p) % 24];
  }
}

float sinewave(int x) {
  return sin(0.2618*x)*(4/2);
}
void initPressuresBetterWithFormular(float pressure) {
  for(int p = 0; p < PRESSURES_SIZE; p++) {
    pressures[p] = pressure + sinewave((hour+p) % 24);
  }
}

void printPressures() {
    for (int i = 0; i < PRESSURES_SIZE; i++) {
    Serial.println(pressures[i]);
    //Serial.print(" ");
  }
  //Serial.println();
}
