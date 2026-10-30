#define PRESSURE_SENSOR_PIN A2

double currentPressure;
double currentPSI;

double getPSI(double pressure){
  double pressure1 = pressure - 73;
  double pressure2 = pressure1 / 8;
  return pressure2;  
}

void setup() {
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  currentPressure = analogRead(PRESSURE_SENSOR_PIN);
  currentPSI = getPSI(currentPressure);
  Serial.println(currentPSI);
}
