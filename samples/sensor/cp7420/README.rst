.. cp7420:

 CP7420 Charger
 ###################################

 Overview
 ********



 Requirements
 ************

 CP7420

 This sample requires a board which provides a configuration for Arduino
 connectors and defines node aliases for the I2C interface.
 For more info about the node structure see
 :zephyr_file:`samples/sensor/cp7420/boards/nucleo_f401re.overlay`

 Building and Running
 ********************

 This sample application uses an CP7420 sensor connected to a board via I2C.
 Connect the sensor pins according to the connection diagram given in the

 .. zephyr-app-commands::
    :zephyr-app: samples/sensor/cp7420
    :board: nucleo_f401re
    :goals: build flash
    :compact:

 Sample Output
 =============
 To check output of this sample , any serial console program can be used.
 This example uses ``picocom`` on the serial port ``/dev/ttyUSB0``:

 References
 ***********