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
  Version: 1.4
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
boolean DEBUG = false;

// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];   // An array to store the received data

// Time for next timeout, in milliseconds
unsigned long nextTimeout = 0;
boolean timerRunning = false;
const int RESET_TIME = 250;

// Constants representing the states in the state machine
const int S_IDLE = 0;
const int S_RUNNING = 1;
int currentState = S_IDLE; // A variable holding the current state

// Defining servo object
const int SERVO_PIN = 9;
Servo servo;

// Defining sensor object
const int SENSOR_PIN = A1;
// Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 15;
SharpDistSensor ir_sensor(SENSOR_PIN, mediumFilterWindowSize);

// Defining PID variables
const double HIGHER_LIMIT = 100.0;
const double LOWER_LIMIT = 0.0;
double kp = 0.2;
double ki = 0.1;
double kd = 0.5;
double actualValue = 0.0;
double setValue = 50.0; // Initial setvalue
double output = 0.0;
PID pid(&actualValue, &output, &setValue, kp, ki, kd, REVERSE);


// Configures hardware (digital outputs) and serial comm.
void setup() {
  Serial.begin(9600); // Starts Serial communication

  servo.attach(SERVO_PIN);
  servo.write(90); // Servo startposition

  // Set sensor model
  ir_sensor.setModel(SharpDistSensor::GP2Y0A41SK0F_5V_DS);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(LOWER_LIMIT, HIGHER_LIMIT);

  startUpMsg();
}

// Main loop
void loop() {
  actualValue = getDistanceInPercent();
  recvWithEndMarker();
  if (DEBUG) {
    changeConfigurations();
  } else if ((!DEBUG) && (Serial.available() > 0)) {
    changeSetvalue();
  }
  switch (currentState) {
    case S_IDLE:
      servo.write(84); // Servo startposition
      if (isBallOn(SENSOR_PIN)) {
        changeStateTo(S_RUNNING);
      }
      break;

    case S_RUNNING:
      updatePosition();
      if ((!isBallOn(SENSOR_PIN)) && (!timerRunning)) {
        startTimer(RESET_TIME);
        timerRunning = true;
      }
      if ((hasTimerExpired()) && (timerRunning)) {
        timerRunning = false;
        changeStateTo(S_IDLE);
      }
      break;

    default:
      servo.detach();
      Serial.println("ERROR, please reboot the device.");
  }
  printSystemStatus();
}

/**
   Checks if the timer has expired. If the timer has expired,
   true is returned. If the timer has not yet expired,
   false is returned.

   @return true if timer has expired, false if not
*/
boolean hasTimerExpired() {
  boolean hasExpired = false;
  if (millis() > nextTimeout) {
    hasExpired = true;
  }
  return hasExpired;
}

/**
  Starts the timer and set the timer to expire after the
  number of milliseconds given by the parameter duration.

  @param duration The number of milliseconds until the timer expires.
*/
void startTimer(unsigned long duration) {
  nextTimeout = millis() + duration;
}

/**
   Fetches the raw sensor value.

   @param pin position of sensor
   @return raw sensorvalue (0 - 1024)
*/
int getRawSensorvalue(int pin) {
  return analogRead(pin);
}

/**
   Fetches the converted
   measured value from sensor

   @return converted measurment in cm
*/
int getDistanceInCm() {
  return ir_sensor.getDist() / 10;
}

/**
   Fetches the converted
   measured value from sensor

   @return converted measurment in %
*/
int getDistanceInPercent() {
  float distance = ir_sensor.getDist();
  distance = constrain(distance, 20, 350);
  distance = map(distance, 20, 350, LOWER_LIMIT, HIGHER_LIMIT);
  return distance;
}

/**
   Checks if the ball is placed
   within the measruement margins.

   @param pin position of sensor
   @return state of the ball (true / false)
*/
boolean isBallOn(int sensorpin) {
  boolean state = false;
  int raw = getRawSensorvalue(sensorpin);
  if ((raw < 900) && (raw > 100)) {
    state = true;
  }
  return state;
}

/**
  Sets the position of the servo to the position
  given by the parameter pos

  @param pos The position to set the servo to in degrees.
  Must be between 0 and 180 degrees
*/
void setServoPos(int pos) {
  pos = constrain(pos, 0, 100);
  pos = map(pos, 0, 100, 80, 100);
  servo.write(pos);
}

/**
   Updates the servoposition
   to the new PID-output.
*/
void updatePosition() {
  pid.Compute();
  setServoPos(output);
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
    receivedChars[ndx] = '\0'; // terminate the string
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
   Prints the state to Serial Monitor as a text, based
   on the state-constant provided as the parameter state

   @param state The state to print the tekst-representation for
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

void changeConfigurations() {
  if (newData) {
    kp = getValue(receivedChars, ':', 0).toDouble();
    ki = getValue(receivedChars, ':', 1).toDouble();
    kd = getValue(receivedChars, ':', 2).toDouble();
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
  Serial.print(125);  // To freeze the upper limit
  Serial.print(" ");
  Serial.print(actualValue);
  Serial.print(",");
  Serial.print(setValue);
  Serial.print(",");
  Serial.println(output);
}

/**
   Prints a startup message to Serial Monitor as a text.
*/
void startUpMsg() {
  Serial.println("<Device up and running>");
}

/**
  Reads a string from Serial Monitor.
*/
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  // if (Serial.available() > 0) {
  while ((Serial.available() > 0) && (!newData)) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
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
String getValue(String data, char separator, int index) {
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
