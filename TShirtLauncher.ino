
/**

  T-Shirt Cannon Code
  Written by: Naman Khurana and Steve King

  Wiring:
    Master Compressor Switch: Pin 2
    LED DATA (PWM): Pin 3
    Arm Safety Button (Deadman): 4
    Left Wheel Motor (PWM): 9
    Right Wheel Motor (PWM): 10
    Fire Left Button: 7
    Fire right Button 8
    Air Compressor Relay Pin: 5
    Fire Left Solenoid Relay: 6
    Fire Right Solenoid Relay: 11
    Ram Rod Interlock Pin: 12

    Drive Joystick X: A0
    Drive Joystick Y: A1
    Pressure Sensor Pin: A2

  Controller Wiring
    Pin 1  White Orange   Left Fire Data
    Pin 2  Orange         Right Fire Data
    Pin 3  White Green    +5 Volts
    Pin 4  Blue           Safety Button Data
    Pin 5  White Blue     Safety Button Ground
    Pin 6  Green          Ground
    Pin 7  White Brown    Joystick X Data
    Pin 8  Brown          Joystick Y Data
*/

// ------ IMPORTS
#include <FastLED.h>
#include <FRCmotor.h>

// ------   C O N S T A N T S   a n d   P I N   A S S I G N M E N T S   ------
// ------ Air Compressor PSI Limits
#define PSI_CUTOFF 70      // PSI at which we shut off the air compressor
#define PSI_START 65       // PSI below which we restart the air compressor
#define SHOOT_DURATION 100 // How long to Keep Firing Solenoid Open in milliseconds
// Buttons
#define SAFETY_BUTTON_PIN 4
#define FIRE_LEFT_BUTTON_PIN 7
#define FIRE_RIGHT_BUTTON_PIN 8
#define FIRE_LEFT_SOLENOID_RELAY 5
#define FIRE_RIGHT_SOLENOID_RELAY 11

// Drive Joystick Input and Caps
#define JOYSTICK_X A0
#define JOYSTICK_Y A1
#define MAX_SPEED 0.5
#define MAX_TURN 0.25

// Compressor
#define MASTER_COMPRESSOR_SWITCH 2
#define AIR_COMPRESSOR_PIN 6
#define PRESSURE_SENSOR_PIN A2

#define RAM_ROD_INTERLOCK_PIN 12

// ------ LED Constants
#define LED_DATA_PIN 3

#define NUM_LEDS 60
#define LED_TYPE WS2811
#define COLOR_ORDER BRG
#define BRIGHTNESS 50
#define TWINKLE_DELAY 20 // Milliseconds between updates
#define GREEN 0x008000
#define RED 0xFF0000
#define YELLOW 0xFFFF00
#define ORANGE 0xFF4500
#define BLUE 0x0000ff
#define TEAL 0x008080

// Joystick Adjustment Constants
#define JOYSTICK_X_CENTER 512
#define JOYSTICK_Y_CENTER 512
#define JOYSTICK_DEADZONE 15

// Wheels
#define LEFT_MOTOR_PIN 10
#define RIGHT_MOTOR_PIN 9

// create servo objects to control the wheel motors
FRCmotor leftSideMotor;
FRCmotor rightSideMotor;

FRCmotor compressor;

enum actionState
{
  POWERED,  // Powered but safety Switch is not pressed
  READY,    // Safety switch depressed so drivable, but Ram rod not stored
  CHARGING, // Not ARMED or FIRING but air compressor is running
  ARMED,    // The Safety switch is depressed and the Ram Rod is stored
  FIRING    // One of the cylinders is Firing
};

typedef struct
{
  int bottomNode; // index number of the 'bottom' pixel in the full leds array
  int nodeCount;  // the number of pixels in this segment
  int direction;  // how to get 'up' to the next pixel in the segment: +1 or -1
} pixelSegment;

// set-up Pixel Legs Array
pixelSegment legs[6] =
    {
        {0, 12, +1},  //  0 - 11  up
        {23, 12, -1}, // 12 - 23  down
        {24, 12, +1}, // 24 - 35  up
                      // 2 pixels to back light sign: 36, 37
        {49, 12, -1}, // 38 - 49  down
        {50, 12, +1}, // 50 - 61  up
        {62, 12, -1}  // 62 - 73  down
};
// ------ VARIABLES
int gamemode = 1; // Needed for FRCMotor, just do it

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
int masterCompressorSwitchStatus;
unsigned long compressorStartTime;
unsigned long currentMillis;
unsigned long lastUpdateMillis;
double currentPressure;
double currentPSI;
int ramRodInterlock;

// --- STATE VARIABLES
bool compressorOn = false;
bool firingLeft = false;
bool firingRight = false;
bool compressorAutoFill = false;
bool compressorCharged = false;
actionState currentState = POWERED;
actionState lastState;

// ====== SETUP
void setup()
{
  delay(3000); // 3 - second delay on start-up to recover

  // -- Set-up Wheel Motors as Servo objects
  // leftSideMotor.attach(LEFT_MOTOR_PIN);
  // rightSideMotor.attach(RIGHT_MOTOR_PIN);
  leftSideMotor.SetPort(LEFT_MOTOR_PIN);
  rightSideMotor.SetPort(RIGHT_MOTOR_PIN);

  compressor.SetPort(AIR_COMPRESSOR_PIN);

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

  pinMode(MASTER_COMPRESSOR_SWITCH, INPUT_PULLUP);
  pinMode(PRESSURE_SENSOR_PIN, INPUT);

  pinMode(RAM_ROD_INTERLOCK_PIN, INPUT);

  Serial.begin(115200);
}

// ====== MAIN LOOP

void loop()
{
  currentMillis = millis();

  // Current Pressure
  currentPressure = analogRead(PRESSURE_SENSOR_PIN);
  currentPSI = getPSI(currentPressure);

  // Read the Buttons and Switches
  masterCompressorSwitchStatus = digitalRead(MASTER_COMPRESSOR_SWITCH);
  safetyButton = digitalRead(SAFETY_BUTTON_PIN);

  fireLeftButton = digitalRead(FIRE_LEFT_BUTTON_PIN);
  fireRightButton = digitalRead(FIRE_RIGHT_BUTTON_PIN);
  ramRodInterlock = digitalRead(RAM_ROD_INTERLOCK_PIN);

  // Respond to State changes

  if ((currentState == ARMED || currentState == FIRING) && fireLeftButton == LOW && firingLeft == false)
  {
    firingLeft = true;
    fireLeftStartTime = currentMillis;
    digitalWrite(FIRE_LEFT_SOLENOID_RELAY, HIGH);
  }

  if (firingLeft && currentMillis > fireLeftStartTime + SHOOT_DURATION)
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

  if (firingRight && currentMillis > fireRightStartTime + SHOOT_DURATION)
  {
    firingRight = false;
    digitalWrite(FIRE_RIGHT_SOLENOID_RELAY, LOW);
  }

  // Master Compressor SWitch Check
  if (masterCompressorSwitchStatus == LOW)
  {
    // REGULAR CYCLE ACTIVITIES
    if (currentPSI <= PSI_START || !compressorCharged)
    {
      // Turn on the Compressor
      compressorOn = true;
      compressor.Set(100);
      compressorStartTime = currentMillis;
    }

    // Turn Compressor Off
    if (currentPSI >= PSI_CUTOFF)
    {
      compressorOn = false;
      compressorCharged = true;
      compressor.Set(0);
    }
  }
  else
  {
    // Compressor Master Switch Off
    compressorOn = false;
    compressor.Set(0);
  }

  // Set state Flag
  if (firingLeft || firingRight)
  {
    currentState = FIRING;
  }
  else if (safetyButton == LOW && ramRodInterlock == LOW)
  {
    currentState = ARMED;
  }
  else if (compressorOn)
  {
    currentState = CHARGING;
  }
  else if (safetyButton == LOW)
  {
    currentState = READY;
  }
  else
  {
    currentState = POWERED;
  }

  // Drive the T-Shirt Cannon
  driveRobot();

  // -- Set LEDs for Current State
  updateLEDs();

  // update Serial Monitor
  // updateLog(); // <------ C O M M E N T   O U T   f o r   P R O D U C T I O N

  // -- CLEAN UP LOOP
  lastState = currentState;
}

// ------ UTILITY FUNCTIONS

/**
 * Routine to read joystick and  set respective drive wheel power
 *
 * Note: uses JOYSTICK_Y_CENTER and JOYSTICK_X_CENTER to find the center joystick
 *       values and uses JOYSTICK_DEADZONE to ignore a range of values around that
 *       center.
 *
 *       JOYSTICK_X has range 0 - 1023
 *         We convert two ranges independently
 *         1) the bottom 0 to JOYSTICK_X_CENTER - JOYSTICK_DEADZONE
 *              to the range -89 to -1, then
 *         2) the top JOYSTICK_X_CENTER + JOYSTICK_DEADZONE to 1023
 *              into the range  1 to 90
 *         the center, plus/minus the deadzone maps to 90
 */
void driveRobot()
{
  xPosition = analogRead(JOYSTICK_X);
  yPosition = analogRead(JOYSTICK_Y); // Y side of Joystick is reversed

  if ((xPosition < JOYSTICK_X_CENTER - JOYSTICK_DEADZONE) && (safetyButton == LOW))
  { // Left Turn
    // ORIGINAL: turn = map(xPosition, 1, JOYSTICK_X_CENTER - JOYSTICK_DEADZONE, -89, -1);
    turn = map(-(sq(xPosition / (float)(JOYSTICK_X_CENTER - JOYSTICK_DEADZONE))), -1, 0, 1, 90) * MAX_TURN;
  }
  else if ((xPosition > JOYSTICK_X_CENTER + JOYSTICK_DEADZONE) && (safetyButton == LOW))
  { // Right Turn
    // ORIGINAL: turn = map(xPosition, JOYSTICK_X_CENTER + JOYSTICK_DEADZONE, 1023, 1, 90);
    turn = map(sq(xPosition / (float)(JOYSTICK_X_CENTER - JOYSTICK_DEADZONE)), 0, 1, 1, 90) * MAX_TURN;
  }
  else
  {
    turn = 0;
  }

  if ((yPosition < JOYSTICK_Y_CENTER - JOYSTICK_DEADZONE) && (safetyButton == LOW))
  {
  // ORIGINAL: drive = map(yPosition, 1, JOYSTICK_Y_CENTER - JOYSTICK_DEADZONE, -89, 1);
    drive = map(-(sq(yPosition / (float)(JOYSTICK_Y_CENTER - JOYSTICK_DEADZONE))), -1, 0, 1, 90) * MAX_SPEED;

  }
  else if ((yPosition > JOYSTICK_Y_CENTER + JOYSTICK_DEADZONE) && (safetyButton == LOW))
  {
  // ORIGINAL: drive = map(yPosition, JOYSTICK_Y_CENTER + JOYSTICK_DEADZONE, 1023, 1, 90);
      drive = map(sq(yPosition / (float)(JOYSTICK_Y_CENTER - JOYSTICK_DEADZONE)), 0, 1, 1, 90) * MAX_SPEED;

  }
  else
  {
    drive = 0;
  }

  leftWheelPower = constrain(drive + turn, -100, 100); // drive + turn +90
  rightWheelPower = constrain(drive - turn, -100, 100);

  // Drive Motor Settings
  // leftSideMotor.write(leftWheelPower);
  // rightSideMotor.write(rightWheelPower);
  leftSideMotor.Set(leftWheelPower);
  rightSideMotor.Set(-rightWheelPower);
}

/**
 *  Routine to write the Serial Monitor lines.  SHOULD NOT
 *        be called in Production
 */
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
  case POWERED:
    Serial.print("POWERED");
    break;
  }

  // Serial.print(safetyButton);
  // Serial.print('\t');
  // Serial.print(fireLeftButton);
  // Serial.print('\t');
  // Serial.print(fireRightButton);
  Serial.print("\txPos: ");
  Serial.print(xPosition);
  Serial.print("\tyPos: ");
  Serial.print(yPosition);
  Serial.print("\tLeft: ");
  Serial.print(leftWheelPower);
  Serial.print("\tRight: ");
  Serial.print(rightWheelPower);
  Serial.print("\t");
  Serial.print(masterCompressorSwitchStatus);
  Serial.print(ramRodInterlock);
  Serial.print(safetyButton);

  Serial.print("\tFire: ");
  if (firingLeft && firingRight)
  {
    Serial.print(" BOTH ");
  }
  else if (firingLeft)
  {
    Serial.print(" LEFT ");
  }
  else if (firingRight)
  {
    Serial.print(" RIGHT");
  }
  else if (currentState == ARMED)
  {
    Serial.print(" ARMED");
  }
  else
  {
    Serial.print("      ");
  }
  Serial.print("\tPSI: ");
  Serial.print(currentPSI);

  Serial.println("");
}

/**
 *  Main routine to set the LEDs based on the current state of the Launcher
 *
 */
void updateLEDs()
{
  switch (currentState)
  {

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
  case CHARGING:
    if (lastState != CHARGING || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      fillLegs();
      lastUpdateMillis = currentMillis;
    }
    break;
  case READY:
    if (lastState != READY || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      fillColorWithGlitter(BLUE);
      lastUpdateMillis = currentMillis;
    }
    break;
  case POWERED:
    if (lastState != POWERED || currentMillis > lastUpdateMillis + TWINKLE_DELAY)
    {
      rainbowWithGlitter();
      lastUpdateMillis = currentMillis;
    }
    break;
  }

  FastLED.show(); // Update the LEDs
}

/**
 * Fill up each leg of the LEDs representing the current Pressure
 *  relative to the cutoff pressure.
 *
 * If below the Compressor Kick-in pressure use Orange
 * once above that value make the fill Yellow
 *
 * Use Blue a background color
 */
void fillLegs()
{
  int currLED;
  uint8_t howHigh = (currentPSI / PSI_CUTOFF) * legs[0].nodeCount;
  uint32_t fillColor;

  if (currentPSI < PSI_START)
  {
    fillColor = ORANGE;
  }
  else
  {
    fillColor = YELLOW;
  }

  fill_solid(leds, NUM_LEDS, BLUE);

  for (uint8_t leg = 0; leg < 6; leg++)
  {
    currLED = legs[leg].bottomNode;
    for (uint8_t i = 0; i < howHigh; i++)
    {
      leds[currLED] = fillColor;
      currLED = currLED + legs[leg].direction;
    }
  }
}

/**
 *  Fill the LEDs with a Color
 *
 * @param colorName
 */
void fillColorWithGlitter(uint32_t colorName)
{
  fill_solid(leds, NUM_LEDS, colorName);
  addGlitter(100);
}

/**
 *  Fill the LEDs with a Rainbow effect
 */
void rainbow()
{
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}
/**
 *  Fill the rainbow effect but add Glitter
 */
void rainbowWithGlitter()
{
  rainbow();
  addGlitter(80);
}

/**
 *  Change random LEDs to White based on the chanceOfGlitter value
 *
 * @param chanceOfGlitter
 */
void addGlitter(fract8 chanceOfGlitter)
{
  if (random8() < chanceOfGlitter)
  {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

/**
 * Calculate the "real" PSI based on the pressure sensor reading
 *
 * @param pressure
 * @return double
 */
double getPSI(double pressure)
{
  return ((pressure - 73) / 8) - 3;
}
