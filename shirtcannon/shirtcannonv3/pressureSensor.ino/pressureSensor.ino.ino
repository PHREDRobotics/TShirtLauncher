#define PRESSURE_SENSOR_PIN A2

double currentPressure;


void setup(){
    pinMode(PRESSURE_SENSOR_PIN, INPUT);
    Serial.begin(9600);
}

void loop(){
    currentPressure = analogRead(PRESSURE_SENSOR_PIN);
    Serial.println(currentPressure);
}
