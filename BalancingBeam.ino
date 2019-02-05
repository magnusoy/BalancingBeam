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
  Code by: Magnus Øye, Dated: 30.01-2019
  Version: 1.6
  Contact: magnus.oye@gmail.com
  Website: https://github.com/magnusoy/BalancingBeam
*/


/**
  HOW IT WORKS:

      If DEBUG is false:
              Open Serial Monitor and write any number
              between 0 - 100 to change setvalue.

              Open Serial Plotter to watch values
              being plotted in real time.

      If DEBUG is true:
              Open Serial Monitor and you will now be
              able to change PID parameters by writing
              in the following format: 1.5:0.6:0.2
              to change kp, ki, kd params.
*/


// Including necessary libraries
#include <PID_v1.h>
#include <Servo.h>
#include <SharpDistSensor.h>

// Turn ON/OFF debug functionality to Serial Monitor
const boolean DEBUG = false;

// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];   // An array to store the received data

// Constants representing the states in the state machine
const int S_IDLE = 0;
const int S_RUNNING = 1;
int currentState = S_IDLE; // A variable holding the current state

// Defining servo object
const int SERVO_PIN = 9;
const int START_POS = 98; // In degrees
const int SERVO_RANGE = 10; // In degrees
const int SERVO_LIMIT_LOW = START_POS + SERVO_RANGE; // In degrees
const int SERVO_LIMIT_HIGH = START_POS - SERVO_RANGE; //In degrees
Servo servo;

// Defining sensor object
const int SENSOR_PIN = A1;
// Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 15;
SharpDistSensor irSensor(SENSOR_PIN, mediumFilterWindowSize);

// Defining PID variables
const double HIGHER_LIMIT = 100.0;
const double LOWER_LIMIT = 0.0;
double kp = 0.42;
double ki = 0.2;
double kd = 0.9;
double actualValue = 0.0;
double setValue = 50.0; // Initial setvalue
double output = 0.0;
PID pid(&actualValue, &output, &setValue, kp, ki, kd, REVERSE);


// Configures hardware (digital outputs) and serial comm.
void setup() {
  Serial.begin(19200); // Starts Serial communication

  servo.attach(SERVO_PIN);
  setDefaultPosition(START_POS);

  // Set sensor model
  irSensor.setModel(SharpDistSensor::GP2Y0A60SZLF_5V);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(LOWER_LIMIT, HIGHER_LIMIT);

  startupMsg();
}

// Main loop
void loop() {
  actualValue = getDistanceInPercent();
  readStringFromSerial();
  if (DEBUG) {
    changeConfigurations();
  } else if ((!DEBUG) && (Serial.available() > 0)) {
    changeSetvalue();
  }
  switch (currentState) {
    case S_IDLE:
      setDefaultPosition(START_POS);
      output = 50;
      if (isBallOn(actualValue)) {
        // Waiting for ball
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
  printSystemStatus();
}

/**
   Fetches the converted
   measured value from sensor

   @return converted measurment in %
*/
int getDistanceInPercent() {
  float distance = irSensor.getDist();
  distance = constrain(distance, 20, 600);
  distance = map(distance, 20, 600, 0, 100);
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
  Fetches an integer number
  from the Serial Terminal.

  @return recieved integer from
          Serial Terminal.
*/
int readNumberFromSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc = Serial.read();

  if (rc != endMarker) {
    receivedChars[ndx] = rc;
    ndx++;
    if (ndx >= numChars) {
      ndx = numChars - 1;
    }
  } else {
    receivedChars[ndx] = '\0'; // Terminate the string
    ndx = 0;
    newData = true;
  }
  return atoi(receivedChars);
}

/**
   Changes the setvalue to the
   value recieved from serial.
*/
void changeSetvalue() {
  int bufferValue = readNumberFromSerial();
  if ((bufferValue <= HIGHER_LIMIT) && (bufferValue >= LOWER_LIMIT)) {
    setValue = bufferValue;
  }
}

/**
  Set default servo position.

   @param position in degrees
*/
void setDefaultPosition(int pos) {
  servo.write(pos);
}

/**
   Prints the state to Serial Monitor as a text, based
   on the state-constant provided as the parameter state

   @param state The state to print the text-representation for
*/
void printState(int state) {
  switch (state) {
    case S_IDLE:
      Serial.print("S_IDLE");
      break;

    case S_RUNNING:
      Serial.print("S_RUNNING");
      break;

    default:
      Serial.print("!!UNKNOWN!!");
      break;
  }
}

/**
  Updates the PID parameters if it detects new
  data in Serial Monitor in the following format->
  1.0:0.2:0.0 where ':' seperates the values.
  Alternative add another ':' to change setValue aswell.
*/
void changeConfigurations() {
  if (newData) {
    kp = getValueFromSerial(receivedChars, ':', 0).toDouble();
    ki = getValueFromSerial(receivedChars, ':', 1).toDouble();
    kd = getValueFromSerial(receivedChars, ':', 2).toDouble();
    if (getValueFromSerial(receivedChars, ':', 3).toDouble() > 1) {
      setValue = getValueFromSerial(receivedChars, ':', 3).toDouble();
    }
    newData = false;
  }
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
   Prints the systemstate to Serial Monitor as a text.
   Dependent on DEBUG to show either information or
   raw values for Serial Plotter.
*/
void printSystemStatus() {
  if (DEBUG) {
    String content = " Actual: " + String(actualValue) +
                     " Set: " + String(setValue) +
                     " Output: " + String(output) +
                     " Params: " + String(kp) +
                     ", " + String(ki) +
                     ", " + String(kd);
    Serial.print("State: ");
    printState(currentState);
    Serial.println(content);
  } else {
    plotValues();
  }
}

/**
  Sends important data to Serial Plotter
  for graphing.
*/
void plotValues() {
  Serial.print(actualValue);
  Serial.print(",");
  Serial.print(setValue);
  Serial.print(",");
  Serial.println(output);
}

/**
   Prints a startup message to Serial Monitor as a text.
*/
void startupMsg() {
  Serial.println("<Device up and running>");
}

/**
  Reads a string from Serial Monitor.
*/
void readStringFromSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while ((Serial.available() > 0) && (!newData)) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

/**
  Fetches the value from a substring,
  wich is seperated with a given symbol.

  @param data your String to be seperated
  @param seperator your symbol to seperate by
  @param index where your value is located

  @return substring before seperator

*/
String getValueFromSerial(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
