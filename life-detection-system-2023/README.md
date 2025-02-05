# Life Detection System - 2023

## Description
The 2023 LD system consisted of 1) a linear actuator, which moved a funnel along 6 different positions corresponding to 6 different soil sample slots, 2) a water pump, and 3) an array of 6 light sensors. It was controlled with an XBox controller.

## Components
1. Firmware - this code worked with a Nucleo L476RG to control a linear actuator, light sensor, and water pump (which was a DC step motor). 
2. ROS Nodes - these nodes interacted with the firmware and an xbox controller, as well as the UI, to provide an interface for life detection system control.
3. UI - there was a dedicated 'science' tab in our UI which displayed the linear actuator position and light sensor readings.

## Firmware - ld_board_2023

### My Contributions
I worked with two other people on the team when setting up the initial core of the project (thread structure, main loop, etc). I then worked on adding different control schemes for the linear actuator (direct velocity control vs pre-set positions), as well as improving light sensor reading timing and accuracy. 

### Development Issues
#### 1: Dropped light sensor request messages
The science team required that they take light sensor readings exactly 2 minutes apart from each other during their testing. This proved to be difficult, as the 'MBedQueue' structures used to store messages to each thread would drop the most recent messages if the queue filled up. This happend quite often, as messages were being sent to the dev board every time the corresponding ROS node 'spun'. 

My solution:  
To prevent duplicated light sensor requests, I added an ID to each request. I also added some state variables to track which light sensor was last requested, and added timestamps to the readings. This is likely not the cleanest solution, but this feature was implemented a few weeks before competition, so I went with the first working implementation. 

Alternative solutions:  
1. Only send commands/requests when needed, not on every 'spin' of the ROS node. This would have prevented the queue from filling up.

#### 2: Linear Actuator potentiometer reading accuracy
The accuracy of the linear actuator's potentiometer reading (which was used to command the linear actuator to pre-set positions) degraded over time. This resulted in issues when the actuator was
commanded to move to the 6th position multiple times in a row (there was a risk of the actuator driving into itself). 
  
My Solution: I tracked whether or not the last commanded position was position 6, and prevented it from being executed multiple times in a row. This was another decision driven by tight time constraints. 

## ROS Nodes
Note - at the time, the team was having issues with getting imports working for custom libraries (`XBoxControllerState` and `RoverSerial` specifically) so the code had to be copy-pasted when used in ROS node files. This has since been resolved.  
  
### ld_board
This node contains most of the tricky, convoluted state logic that arose from the previous issues mentioned. It's not clean, but it worked during competition, and a lot of it was written in Utah during the competition itself. 

### ld_joy_control
This node controls the life detection system using joystick inputs. Specifically, it uses an `XBoxControllerState` class to handle monitoring and responding to changes in the XBox controller state. I did not write this code, I merely used it inside of the ros node to control the system.


## UI
I wrote the 'science' tab of our UI. 

### Notes
Our UI uses custom react hooks to publish/subscribe to ROS topics (the `useRosTopic` hook you can see used in some of the .tsx files).
