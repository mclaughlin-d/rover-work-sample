# rover-work-sample

This repo contains samples of work I have done for the Mars Rover Team. Each folder contains a project I had a large amount of ownership
over. My specific contributions, as well as explanations for choices I made, are detailed in README files inside each folder.

## Rover-specific Notes
The Rover team uses some specific libraries and custom pieces of code which are not included in this repo but I use in my samples. I included some brief explanations of what they are below. 

### RoverSerial
The Rover team has worked with a custom serial protocol, called RoverSerial, built on top of MBedOS's serial protocols (USBSerial and UnbufferedSerial). This library works by defining a single 'struct' message type for each direction of the serial communication, which is the only type of message that can be sent over serial.

### XBoxControllerState
The Rover team uses a custom utility class (not written by me) to use joysticks in our ROS layer.
