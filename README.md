# BalancingBeam
An Arduino is used to balance a ball rolling on a beam. It uses an IR-Sensor for measurment and a PID-Controller controls the servo's output for lift.

### Prerequisites

You will need [Arduino IDE](https://www.arduino.cc/en/Main/Software) to upload the code over to the Arduino Uno/Nano board.


### Installing

Download .zip or clone repository to your computer.

The code utilize a number of open source libraries wich is included under **/libraries.**

The following libraries that are included:
* [PID](https://github.com/br3ttb/Arduino-PID-Library) - PID
* [Servo](https://github.com/arduino-libraries/Servo) - Servo
* [SharpDistSensor](https://github.com/DrGFreeman/SharpDistSensor) - SharpDistSensor
* [Filters](https://github.com/JonHub/Filters) - Filters

Copy all of the content in the /libraries folder and move them over to your own Arduino libraries folder.
This folder will often be in your Documents/Arduino folder.

You will now be able to open and run the project.

![Output](https://github.com/magnusoy/BalancingBeam/blob/master/docs/connections.png)

## To be noted
There are several constants and parameters that can vary. Therefor be aware that you might need to change the following to get it working:

- **Sensor pin** connected to A1
- **Servo pin** connected to D9
- **START_POS** should be at an angle where the beam is parallel to the ground
- **SERVO_RANGE** should be at an angle for good control, play around and check
- **MIN_DISTANCE** should be where you want the sensor to start measure distance
- **MAX_DISTANCE** should be where you want the sensor to stop measure distance
- **mediumFilterWindowSize** has to be an odd number and you should scale it up until noise is removed
- **kp, ki, kd** is the PID params and the numbers depend on your construction, play around and see
- **irSensor.setModel(SharpDistSensor::GP2Y0A60SZLF_5V)** be sure to select the right model

### Usage

 #### If DEBUG is false:
  Open Serial Monitor and write any number
  between 0 - 100 to change setvalue.

  Open Serial Plotter to watch values
  being plotted in real time.


![Output](https://github.com/magnusoy/BalancingBeam/blob/master/docs/ki_adapt.png)


#### If DEBUG is true:
 Open Serial Monitor and you will now be
 able to change PID parameters by writing
 in the following format: **1.5:0.6:0.2**
 to change kp, ki, kd params.


![Output](https://github.com/magnusoy/BalancingBeam/blob/master/docs/status.JPG)


## Built With

* [Arduino](https://www.arduino.cc/) - Arduino

## Contributing

If you want to contribute or find anything wrong, please create a Pull request, or issue adressing the change, or issue.


## Author

* **Magnus Øye** - [magnusoy](https://github.com/magnusoy)


## License

This project is licensed under the MIT License - see the [LICENSE.md](https://github.com/magnusoy/BalancingBeam/blob/master/LICENSE) file for details
