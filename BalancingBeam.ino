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
  Code by: Magnus Øye, Dated: 27.01-2019
  Contact: magnus.oye@gmail.com
  Website: https://github.com/magnusoy/BalancingBeam
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
const int SERVO_PIN = 10;
Servo servo;

// Defining sensor object
const int SENSOR_PIN = A0;
// Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 5;
SharpDistSensor ir_sensor(SENSOR_PIN, mediumFilterWindowSize);

// Defining PID variables
const double HIGHER_LIMIT = 100.0;
const double LOWER_LIMIT = 0.0;
const double Kp = 2.5;
const double Ki = 0.0;
const double Kd = 1.0;
double actualValue = 0.0;
double setValue = 50.0; // Initial setvalue
double output = 0.0;
PID pid(&actualValue, &output, &setValue, Kp, Ki, Kd, DIRECT);


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

//|===============|\\
//|     MAIN      |\\
//|===============|\\

void loop() {
  actualValue = getDistanceInPercent();
  if (Serial.available() > 0) {
    changeSetvalue();
  }
  switch (currentState) {
    case S_IDLE:
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


//|===============|\\
//|   FUNCTIONS   |\\
//|===============|\\


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
  pos = map(pos, 0, 100, 30, 150);
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
int readFromSerial() {
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
  newData = false;
  return atoi(receivedChars);
}

/**
   Changes the setvalue to the
   value recieved from serial.
*/
void changeSetvalue() {
  int bufferValue = readFromSerial();
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

/**
   Change the state of the statemachine to the new state
   given by the parameter newState

   @param newState The new state to set the statemachine to
*/
void changeStateTo(int newState) {
  if (DEBUG) {
    Serial.print("State changed from ");
    printState(currentState);
    Serial.print(" to ");
    printState(newState);
    Serial.println();
  }
  currentState = newState;
}

/**
   Prints the systemstate to Serial Monitor as a text.
*/
void printSystemStatus() {
  String content = " Actual: " + String(actualValue) +
                   " Set: " + String(setValue) +
                   " Output: " + String(output);
  Serial.print("State: ");
  printState(currentState);
  Serial.println(content);
}


/**
   Prints a startup message to Serial Monitor as a text.
*/
void startUpMsg() {
  Serial.println("######################################################");
  Serial.println("<Device up and running>");
  Serial.println("<Everything is standarized to 0 - 100%>");
  Serial.println("<Write in Serial Monitor to change setvalue>");
  Serial.println("<Example:\n       Write 75 to change setvalue to 75%>");
  Serial.println("######################################################");
}
