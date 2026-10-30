/*TODO:
 *-controls
 *  On Bot
 *  -power
 *  -ramrod safety
 *  -manual fire
 *  -lights
 *  Controller
 *  -fire
 *  -move
 *  -deadman's switch
 *  -horn
 *-firing
 *  two person safety
 *-driving 
 *-automated aiming
 *  motor for elevation
 *  targeting
 *  range finding
 *Has Done
 *Fires at 70psi on 12v DC, 
 *hearing protection recommended over 30psi 
 *Simplify/delete power inverter
 *Measured Amps, Volts and PSI comparasion
 *  Highest Amps: 22.5
 *  Used 5% large battery to get to 60PSI
 *Coded firing  
 *Next time:
 *  Mount power supply
 *  Wiring
 *  -new relays
 *  
 *  
 *Battery Volt Amp table
 *,Large battery,,,,,,,,,,,
Amps,0,19.5,21.7,22.5,21.7,22.2,22.3,22.36,22.36,22.04,22.05,
PSI,0,15,20,25,30,35,40,45,50,55,60,recovery
Volts,12.2,11.75,11.6,11.65,11.64,11.64,11.64,11.64,11.64,11.63,11.61,12.1
,(resting start),,,,,,,,,,,
,,,,,,,,,,,,
Solenoid,,,,,,,,,,,,
Volts,12,24,,,,,,,,,,
Amps,0.36,0.7,,,,,,,,,,
 */

const int rightSwitch = 12;
const int leftSwitch = 11;
const int deadSwitch = 13;
const int redPin = 6;
const int greenPin = 5;
const int bluePin = 3;

int rightSwitchState = 0;
int leftSwitchState = 0;
int deadSwitchState = 0;
 
void setup() {
  // put your setup code here, to run once:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(rightSwitch, INPUT);
  pinMode(leftSwitch, INPUT);
  pinMode(deadSwitch, INPUT);
  
  displayColor(0,0,0);
  Serial.begin (9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  rightSwitchState = digitalRead(rightSwitch);
  leftSwitchState = digitalRead(leftSwitch);
  deadSwitchState = digitalRead(deadSwitch);
 
  if (rightSwitchState == HIGH && deadSwitchState == HIGH) {
    displayColor(255,255,0);
    delay(1000);
    displayColor(0,255,255);
  } else {
    displayColor(0,255,255);
  }
  
  if (leftSwitchState == HIGH && deadSwitchState == HIGH) {
    displayColor(255,0,255);
    delay(1000);
    displayColor(0,255,255);
  } else {
    displayColor(0,255,255);
  }

  
}

void displayColor(byte r,byte g,byte b){
  analogWrite(redPin, ~r);
  analogWrite(greenPin, ~g);
  analogWrite(bluePin, ~b);
}
