

const int redPin = 6;
const int greenPin = 5;
const int bluePin = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  displayColor(0,0,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  displayColor(0,255,255);
  delay(500);
  displayColor(255,0,255);
  delay(500);
  displayColor(255,255,0);
  delay(500);
  displayColor(0,255,0);
  delay(500);
  displayColor(0,0,255);
  delay(500);
  displayColor(255,0,0);
  delay(500);
  displayColor(0,0,0);
  delay(500);
  displayColor(255,255,255);
  delay(500);
}

void displayColor(byte r,byte g,byte b){
  analogWrite(redPin, ~r);
  analogWrite(greenPin, ~g);
  analogWrite(bluePin, ~b);
}
