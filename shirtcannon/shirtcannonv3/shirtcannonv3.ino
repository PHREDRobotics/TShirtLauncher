 /**

  T-Shirt Cannon Code
  Written by: Naman Khurana and Steve King

  Wiring:
    LED DATA (PWM): Pin 3
    Arm Safety Button (Deadman): 4
    Left Wheel Motor (PWM): 5
    Right Wheel Motor (PWM): 6
    Fire Left Button: 7
    Fire right Button 8
    Air Compressor Relay Pin: 9
    Fire Left Solenoid Relay: 10
    Fire Right Solenoid Relay: 11
    Ram Rod Interlock Pin: 12

    Drive Joystick X: A0
    Drive Joystick Y: A1
    Pressure Sensor Pin: A2
*/

// ------ IMPORTS
#include <FastLED.h>
#include <Servo.h>

// ------   C O N S T A N T S   a n d   P I N   A S S I G N M E N T S   ------
// ------ Air Compressor PSI Limits
#define PSI_CUTOFF 30 // PSI at which we shut off the air compressor
#define PSI_START 20  // PSI below which we restart the air compressor

// Buttons
#define SAFETY_BUTTON_PIN 4
#define FIRE_LEFT_BUTTON_PIN 7
#define FIRE_RIGHT_BUTTON_PIN 8
#define FIRE_LEFT_SOLENOID_RELAY 10
#define FIRE_RIGHT_SOLENOID_RELAY 11

// Drive Joystick Input
#define JOYSTICK_X A0
#define JOYSTICK_Y A1

// Joystick Tuning Constants
#define JOYSTICK_X_CENTER 560
#define JOYSTICK_Y_CENTER 515
#define JOYSTICK_DEADZONE 4

// Wheels
#define LEFT_MOTOR_POWER 5
#define RIGHT_MOTOR_POWER 6

// Compressor
#define AIR_COMPRESSOR_RELAY_PIN 9
#define PRESSURE_SENSOR_PIN A2

#define RAM_ROD_INTERLOCK_PIN 12

// ------ LED Constants
#define LED_DATA_PIN 3

#define NUM_LEDS 60
#define LED_TYPE WS2811
#define COLOR_ORDER RGB
#define BRIGHTNESS 50
#define TWINKLE_DELAY 20 // Milliseconds between updates
#define GREEN 0x008000
#define RED 0xFF0000
#define YELLOW 0xFFFF00

enum actionState
{
  READY,
  CHARGING,
  ARMED,
  FIRING
};

// ------ VARIABLES
int xPosition = 0;
int yPosition = 0;
int leftWheelPower = 0;
int rightWheelPower = 0;
int turn = 0;
int drive = 0;
CRGB leds[NUM_LEDS];
uint8_t gHue = 0;
int safetyButton = 0;
int fireLeftButton = 0;
int fireRightButton = 0;
unsigned long fireRightStartTime;
unsigned long fireLeftStartTime;
unsigned long compressorStartTime;
unsigned long currentMillis;
unsigned long lastUpdateMillis;
double currentPressure;
double currentPSI;
int ramRodInterlock;
Servo cannon;

// --- STATE VARIABLES
bool compressorOn = false;
bool firingLeft = false;
bool firingRight = false;
bool compressorAutoFill = false;
actionState currentState = READY;
actionState lastState;

// ====== SETUP
void setup()
{
  delay(3000); // 3 - second delay on start-up to recover

  // -- Set-up LEDs
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);

  //  -- Set-up Arduino Pin Assignments
  pinMode(SAFETY_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FIRE_LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(FIRE_RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);

  pinMode(FIRE_LEFT_SOLENOID_RELAY, OUTPUT);
  pinMode(FIRE_RIGHT_SOLENOID_RELAY, OUTPUT);
  pinMode(LEFT_MOTOR_POWER, OUTPUT);
  pinMode(RIGHT_MOTOR_POWER, OUTPUT);

  pinMode(AIR_COMPRESSOR_RELAY_PIN, OUTPUT);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);

  pinMode(RAM_ROD_INTERLOCK_PIN, INPUT);

  Serial.begin(9600);
}

// ====== MAIN LOOP

void loop()
{
  currentMillis = millis();

  // Current Pressure
  currentPressure = analogRead(PRESSURE_SENSOR_PIN);
  currentPSI = getPSI(currentPressure);

  // Read the Buttons and Switches
  safetyButton = digitalRead(SAFETY_BUTTON_PIN);

  fireLeftButton = digitalRead(FIRE_LEFT_BUTTON_PIN);
  fireRightButton = digitalRead(FIRE_RIGHT_BUTTON_PIN);
  ramRodInterlock = digitalRead(RAM_ROD_INTERLOCK_PIN);

  // Set Compressor Relay based on pressure sensor reading
  //   Note: There is a manual switch on the T-Shirt Cannon in series with the relay
  //         If the compressor doesn't cycle check that.

  if (currentPSI <= PSI_START)
  {
    // Turn on the Compressor
    compressorOn = true;
    digitalWrite(AIR_COMPRESSOR_RELAY_PIN, HIGH);
    compressorStartTime = currentMillis;
    currentState = CHARGING;
  }

  // Turn Compressor Off
  if (currentPSI >= PSI_CUTOFF)
  {
    compressorOn = false;
    digitalWrite(AIR_COMPRESSOR_RELAY_PIN, LOW);
    currentState = READY;
  }

  if ((currentState == ARMED || currentState == FIRING) && fireLeftButton == LOW && firingLeft == false)
  {
    firingLeft = true;
    fireLeftStartTime = currentMillis;
    digitalWrite(FIRE_LEFT_SOLENOID_RELAY, HIGH);
  }

  if (firingLeft && currentMillis > fireLeftStartTime + 1000)
  {
    firingLeft = false;
    digitalWrite(FIRE_LEFT_SOLENOID_RELAY, LOW);
  }

  if ((currentState == ARMED || currentState == FIRING) && fireRightButton == LOW && firingRight == false)
  {
    firingRight = true;
    fireRightStartTime = currentMillis;
    digitalWrite(FIRE_RIGHT_SOLENOID_RELAY, HIGH);
  }

  if (firingRight && currentMillis > fireRightStartTime + 1000)
  {
    firingRight = false;
    digitalWrite(FIRE_RIGHT_SOLENOID_RELAY, LOW);
  }

  if (firingLeft || firingRight)
  {
    currentState = FIRING;
  }
  else if (compressorOn)
  {
    currentState = CHARGING;
  }
  else if (safetyButton == LOW && ramRodInterlock == LOW)
  {
    currentState = ARMED;
  }
  else
  {
    currentState = READY;
  }

  // REGULAR CYCLE ACTIVITIES

  // Drive the T-Shirt Cannon
  xPosition = analogRead(JOYSTICK_X);
  yPosition = analogRead(JOYSTICK_Y);
  turn = map(xPosition - JOYSTICK_X_CENTER, -512, 512, -90, 90);
  drive = map(yPosition - JOYSTICK_Y_CENTER, -512, 512, -90, 90);
  leftWheelPower = constrain(-drive + turn + 90, 1, 180);
  rightWheelPower = constrain(-drive - turn + 90, 1, 180);

  // Drive Motor Settings
  if (leftWheelPower < 90 - JOYSTICK_DEADZONE || leftWheelPower > 90 + JOYSTICK_DEADZONE)
  {
    analogWrite(LEFT_MOTOR_POWER, leftWheelPower);
  }
  else
  {
    analogWrite(LEFT_MOTOR_POWER, 90);
  }

  if (rightWheelPower < 90 - JOYSTICK_DEADZONE || rightWheelPower > 90 + JOYSTICK_DEADZONE)
  {
    analogWrite(RIGHT_MOTOR_POWER, rightWheelPower);
  }
  else
  {
    analogWrite(RIGHT_MOTOR_POWER, 90);
  }

  // -- Set LEDs for Current State
  updateLEDs();

  // update Serial Monitor
  updateLog();

  // -- CLEAN UP LOOP
  lastState = currentState;
}

// ------ UTILITY FUNCTIONS

void updateLog()
{
  // Log Conditions
  switch (currentState)
  {
  case READY:
    Serial.print("READY");
    break;
  case ARMED:
    Serial.print("ARMED");
    break;
  case FIRING:
    Serial.print("FIRING");
    break;
  case CHARGING:
    Serial.print("CHARGING");
    break;
  }
  if (safetyButton != HIGH)
  {
    Serial.print("\tPressed");
  }
  else
  {
    Serial.print("\tOff    ");
  }
  Serial.print("\txPos:");
  Serial.print(xPosition);
  Serial.print("\tyPos:");
  Serial.print(yPosition);
  Serial.print("\tTurn:");
  Serial.print(turn);
  Serial.print("\tDrive:");
  Serial.print(drive);
  Serial.print("\tLeft:");
  Serial.print(leftWheelPower);
  Serial.print("\tRight:");
  Serial.print(rightWheelPower);
  Serial.print("\tPSI:");
  Serial.print(currentPSI);

  Serial.println("");
}

void updateLEDs()
{
  switch (currentState)
  {
  case CHARGING:
    if (lastState != CHARGING || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      fillColorWithGlitter(YELLOW);
      lastUpdateMillis = currentMillis;
    }
    break;
  case ARMED:
    if (lastState != ARMED || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      fillColorWithGlitter(RED);
      lastUpdateMillis = currentMillis;
    }
    break;
  case FIRING:
    if (lastState != FIRING || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      fillColorWithGlitter(GREEN);
      lastUpdateMillis = currentMillis;
    }
    break;
  default: // Assume READY
    if (lastState != READY || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      rainbowWithGlitter();
      lastUpdateMillis = currentMillis;
    }
    break;
  }

  FastLED.show(); // Update the LEDs
}

void fillColorWithGlitter(uint32_t colorName)
{
  fill_solid(leds, NUM_LEDS, colorName);
  addGlitter(100);
}

void rainbow()
{
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  rainbow();
  addGlitter(80);
}

void addGlitter(fract8 chanceOfGlitter)
{
  if (random8() < chanceOfGlitter)
  {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

double getPSI(double pressure)
{
  return ((pressure - 73) / 8) - 3;
}
