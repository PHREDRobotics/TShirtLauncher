#define AIR_COMPRESSOR_PIN 9

void setup() {
  pinMode(AIR_COMPRESSOR_PIN, OUTPUT);
  Serial.begin(9600);
  
}

void loop() {
  analogWrite(AIR_COMPRESSOR_PIN, 180);

}
