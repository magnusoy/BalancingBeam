# BalancingBeam
An Arduino is used to balance a ball rolling on a beam. It uses an IR-Sensor for measurment and a PID-Controller controls the servo's output for lift.

### Prerequisites

You will need [Arduino IDE](https://www.arduino.cc/en/Main/Software) to upload the code over to the Arduino Uno/Nano board.


### Installing

The code utilize a number of open source libraries wich is included under /libraries.

The following libraries that are included:
* [PID](https://github.com/br3ttb/Arduino-PID-Library) - PID
* [Servo](https://github.com/arduino-libraries/Servo) - Servo
* [SharpDistSensor](https://github.com/DrGFreeman/SharpDistSensor) - SharpDistSensor

Copy all of the content in the /libraries folder and move them over to your own Arduino libraries folder.
This folder will often be in your Documents/Arduino folder.

You will now be able to open and run the script.

### Usage

Be sure your connections are connected right on the board and press Upload in Arduino IDE.
Then open Serial Monitor and you will be presented with the following information:

 #### If DEBUG is false:
  Open Serial Monitor and write any number
  between 0 - 100 to change setvalue.

  Open Serial Plotter to watch values
  being plotted in real time.


![Output](https://github.com/magnusoy/BalancingBeam/blob/master/docs/plot.JPG)


#### If DEBUG is true:
 Open Serial Monitor and you will now be
 able to change PID parameters by writing
 in the following format: 1.5:0.6:0.2
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
