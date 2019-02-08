/**
  This program is used to balance
  a ball rolling on a beam. It uses
  an IR-Sensor for measurment and a
  PID-Controller controls the servo's output
  for lift.
  -----------------------------------------------------------
  Libraries used:
  PID - https://github.com/br3ttb/Arduino-PID-Library
  Servo - https://github.com/arduino-libraries/Servo
  IR-Sensor - https://github.com/DrGFreeman/SharpDistSensor
  -----------------------------------------------------------
  Code by: Magnus Øye, Dated: 08.02-2019
  Version: 1.2
  Contact: magnus.oye@gmail.com
  Website: https://github.com/magnusoy/BalancingBeam
*/

// Including necessary libraries
#include <PID_v1.h>
#include <Servo.h>
#include <SharpDistSensor.h>

// Constants representing the states in the state machine
const int S_IDLE = 0;
const int S_RUNNING = 1;
int currentState = S_IDLE; // A variable holding the current state

// Defining servo object
const int SERVO_PIN = 9;
const int START_POS = 94; // In degrees
const int SERVO_RANGE = 20; // In degrees
const int SERVO_LIMIT_LOW = START_POS + SERVO_RANGE;  // In degrees
const int SERVO_LIMIT_HIGH = START_POS - SERVO_RANGE; //In degrees
Servo servo;

// Potensiometer for controlling the setvalue
const int POT_PIN = A0;
// Defining sensor object
const int SENSOR_PIN = A1;
const int MIN_DISTANCE = 20; // In millimeters
const int MAX_DISTANCE = 600; // In millimeters
// Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 15;
SharpDistSensor irSensor(SENSOR_PIN, mediumFilterWindowSize);

// Defining PID variables
const double HIGHER_LIMIT = 100.0;
const double LOWER_LIMIT = 0.0;
const double KP = 0.5;
const double KI = 1.2;
const double KD = 0.6;
double actualValue = 0.0;
double setValue = 50.0; // Initial setvalue
double output = 0.0;
PID pid(&actualValue, &output, &setValue, KP, KI, KD, REVERSE);


// Configures hardware (digital outputs) and serial comm.
void setup() {
  servo.attach(SERVO_PIN);
  setDefaultPosition(START_POS);

  // Set sensor model
  irSensor.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(LOWER_LIMIT, HIGHER_LIMIT);
}

// Main loop
void loop() {
  actualValue = getDistanceInPercent(MIN_DISTANCE, MAX_DISTANCE);
  changeSetvalue(POT_PIN);
  switch (currentState) {
    case S_IDLE:
      setDefaultPosition(START_POS);
      if (isBallOn(actualValue)) {
        changeStateTo(S_RUNNING);
      }
      break;

    case S_RUNNING:
      updatePosition();
      if (!isBallOn(actualValue)) {
        changeStateTo(S_IDLE);
      }
      break;

    default:
      servo.detach();
  }
}

/**
   Fetches the converted
   measured value from sensor.

   @param low the min distance in mm
   @param high the max distance in mm
   
   @return converted distance in %
*/
int getDistanceInPercent(int low, int high) {
  float distance = irSensor.getDist();
  distance = constrain(distance, low, high);
  distance = map(distance, low, high, 0, 100);
  return distance;
}

/**
   Checks if the ball is placed
   within the measruement margins.

   @param pin position of sensor
   @return state of the ball (true / false)
*/
boolean isBallOn(float currentPosition) {
  boolean state = false;
  if (currentPosition < 99) {
    state = true;
  }
  return state;
}

/**
  Recieves a % between 0 - 100 and sets the
  servo position between the limits.

  @param pos The position to set the servo to in %
  @param limitLow The lowest it can go in degrees
  @param limitHigh The highest it can go in degrees
*/
void setServoPos(float pos, int limitLow, int limitHigh) {
  pos = constrain(pos, 0, 100);
  pos = map(pos, 0, 100, limitLow, limitHigh);
  servo.write(pos);
}

/**
   Updates the servoposition
   to the new PID-output.
*/
void updatePosition() {
  pid.Compute();
  setServoPos(output, SERVO_LIMIT_LOW, SERVO_LIMIT_HIGH);
}

/**
  Set default servo position,
  and PID output.

   @param position in degrees
*/
void setDefaultPosition(int pos) {
  servo.write(pos);
  output = 50;
}

/**
   Change the state of the statemachine to the new state
   given by the parameter newState

   @param newState The new state to set the statemachine to
*/
void changeStateTo(int newState) {
  currentState = newState;
}

/**
  Change setvalue with potensiometer.

    @param potensiometer pin
*/
void changeSetvalue(int pin) {
  int raw = analogRead(pin);
  int scaled = map(raw, 0, 1024, 0, 100);
  setValue = scaled;
}
