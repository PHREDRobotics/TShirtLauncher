#define MAGNETIC_LIMIT_SWITCH_PIN 9

int switchResult;

void setup() {
  pinMode(MAGNETIC_LIMIT_SWITCH_PIN, INPUT);
  Serial.begin(9600);
}

void loop() {
  switchResult = digitalRead(MAGNETIC_LIMIT_SWITCH_PIN);
  if (switchResult == HIGH){
    Serial.println("HIGH");
  }else if (switchResult == LOW){
    Serial.println("LOW");
  } else {
    Serial.println("NO DATA");
    }

}
