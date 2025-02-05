# LD and Auger Firmware

## Background

This project runs on a Nucleu L476RG board. It controls two linear actuators and two
drive motors, all of which are used in the 2024 auger design for LD. The linear actuators are controlled directly, while the drive motors are controlled using SparkMinis. This project uses RoverSerial to receive commands detailing the speed at which all the components should be moving. It does not send any data back at the moment.

## Testing

This code was tested by plugging in the dev board to a computer (any one is fine as long as it has our repo on it), starting the corresponding ROS2 node, and publishing
to the appropriate topics.
