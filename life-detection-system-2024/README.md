# Life Detection System - 2024

## Description
The life detection system for 2024 consisted of 2 main components: a 3-stage auger which would drill into the dirt to colelct soil samples, and a carousel which performed tests on soil samples. 

## Components
1. Firmware - 'auger_2024': this project controlled a 3-stage auger system as well as a temperature sensor and a humidity sensor.

2. ROS2 Package - 'life_detection': this package has nodes which control a single Dynamixel actuator (carousel_control.py), a NucleoL476RG Dev board (ld_board.py), and a node which interfaces with a joystick to control the system ('life_detection_joy.py). There is a fourth ROS2 node which I did not write.

## Firmware Contributions
For the initial firmware project, I helped a new member write the core of the auger control, which basically involved initializing the 3 pwm outputs for the 3 auger stages, and writing the commanded speeds to each stage in the main loop. I took complete ownership over integrating the sensor(s) (more on that below).

### Development Issues
#### 1: The 'Dallas One-Wire Protocol'
Initially, this project worked with a combined temperature and humidity sensor which was controlled over I2C. However, this sensor broke ~1 week before the team left for Utah, so a replacement was ordered. The humidity sensor was a simple analog input, however, the temperature sensor ordered used the 'Dallas One-Wire Protocol', which MbedOS has not supported since Mbed2 (we used Mbed6).   
  
Solution:  
Due to the tight time constraints, the quick solution was to use a known working board (an Arduino mega) with some modified sample code to read from the sensor. This Arduino Mega was connected to the Nucleo board over serial (which can be seen in the temperature thread in the 'threads' folder). This had to be done due to limited USB slots available on the USB hubs connected to the Jetson Orin (there were no free slots available for interfacing with the Arudio directly). 
  

Takeaways:
1. Communication: more communication between mechanical/science members and myself would have prevented a sensor from being ordered which was not supported by our firmware build system.

## ROS2 Contributions
### carousel_control.py
This node controlled a single Dynamixel which rotated the carousel. It connected with a U2D2 controller over serial, and used the DynamixelSDK to read from and write to the Dynamixel. 
  
Takeaways:  
1. Possible abstraction: this node duplicated a lot of code written for the dynamixel control node (dynamixel_control.py). A future improvement would be to abstract the constants and read/write (as well as group bulk read/write) into a single utility class that could be instantiated to work with different U2D2 boards by different ROS nodes.

### ld_board.py
This node interacted with the devboard which controlled the auger.

### life_detection_joy.py
This node responded to changes in an XBox controller state to control the life detection system. 
