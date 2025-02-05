# Dynamixel Work

## Description
This node controls 3-5 Dynamixels that are daisy-chained together and connected to a U2D2 control board. It uses the DynamixelSDK to read from and write to the motors. 

## My Contributions
### 1) BulkRead/Write
I changed the communication protocol used from direct read/write to bulk read/write, which drastically reduced the amount of duplicated code in the file (I was able to eliminate hundreds of lines of code related to duplicating write commands to each Dynamixel in the chain).

### 2) Offsets
For compatibility with our positional control & IK solvers, I added in offsets to the Dynamixel command values. These are calculated by taking the initial position at start-up as the '0' angle position, and factor these in when publishing the degree positions of each joint in the end effector (wrist pitch, wrist roll, gripper).

### 3) Pitch/Roll Control
I added in pitch/roll control for the differential wrist, instead of relying on individual motor control for each Dynamixel that controls the wrist (which was unintuitive). 

### 4) Optional Typing Dynamixel
For the 2024 URC competition, a typing/allen-key screwing tool was attached to the wrist. This tool used a Dynamixel. The inclusion of this Dynamixel is optional, and specified by launchfile arguments.

### 5) ROS2 Port
I ported this node to ROS2 (previously, this was a ROS node).

### 6) Carousel Dynamixel
Initially, this Dynamixel was controlled by a separate ROS node which connected to a different physical U2D2 board. However, this board broke in Utah the day before the life detection competition. I quite literally patched this in the car on the way to the competition by moving the carousel dynamixel control code to this node, which is why that code is duplicated between the life detection (2024) project and this file.