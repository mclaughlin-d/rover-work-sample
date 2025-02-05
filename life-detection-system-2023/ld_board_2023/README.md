# Description

This project runs on an L476RG, and reads from a light-to-frequency converter (datasheet [here](https://ams.com/documents/20143/36005/TSL237_DS000156_3-00.pdf)).

The firmware counts how many pulses are received in x seconds, converts that to a pulses/sec frequency, and sends it to the board via RoverSerial.


# Testing

This firmware was tested by wiring a light sensor to a dev board (ground to ground, Vdo to 5v, out to an AnalogIn pin) and running `roverserial_test.py` to see the frequencies produced under different circumstances (covering the sensor to block the light, shining a flashlight at the sensor, etc.)

