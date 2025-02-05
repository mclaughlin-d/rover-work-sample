#!/usr/bin/env python3

import rospy
import pathlib
from std_msgs.msg import Bool, Int16, Float32
import struct
from typing import List
from enum import Enum
import threading
import serial
from datetime import datetime
from rospy import Time
import numpy as np


class Struct(object):
    def __init__(self):
        self.size = len(self.toBytes())

    def getPropertyList(self):
        # raise NotImplementedError
        return []

    def toBytes(self):
        bytestring = bytearray()

        for prop in self.getPropertyList():
            value = getattr(self, prop)
            bytestring.extend(self.valToBytes(value))

        return bytestring

    def valToBytes(self, val):
        t = type(val)

        if t is float:
            return struct.pack("f", val)
        elif t is int:
            return struct.pack("i", val)
        elif t is bool:
            return struct.pack("B", val)
        elif t is list:
            bytestring = bytearray()

            for elem in val:
                bytestring.extend(self.valToBytes(elem))

            return bytestring

    def bytesToVal(self, val, bytestring):
        t = type(val)

        if t is float:
            return struct.unpack("f", bytestring[:4])[0], 4
        elif t is int:
            return struct.unpack("i", bytestring[:4])[0], 4
        elif t is bool:
            return struct.unpack("B", bytestring[:1])[0], 1
        elif t is list:
            l = []
            size = 0
            for elem in val:
                val, s = self.bytesToVal(elem, bytestring[size:])
                l.append(val)
                size += s

            return l, size

    def fromBytes(self, input_bytes):
        bytestring = input_bytes[:]

        for prop in self.getPropertyList():
            val, size = self.bytesToVal(getattr(self, prop), bytestring)

            bytestring = bytestring[size:]  # cut off the bottom bytes

            setattr(self, prop, val)


class NoneStruct(Struct):
    def __init__(self):
        super(NoneStruct, self).__init__()

    def getPropertyList(self):
        return []


class RoverSerial:

    transmit_header = b"\xAA\xAA\xFF\xFF"
    receive_header = b"\xBB\xBB\xFF\xFF"

    STATE_LOOK_FOR_HEADER = 0
    STATE_READING_DATA = 1

    def __init__(self, port, baud, Tx_type, Rx_type, receive_cb=None):
        self._ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)
        self._connected = True

        self.Tx_type = Tx_type
        self.Rx_type = Rx_type

        self._rx_bytes = bytearray()
        self._rx_data = self.Rx_type()
        self._rx_header = bytearray([0 for x in range(len(RoverSerial.receive_header))])

        self._readable = False
        self._receive_cb = receive_cb
        self._state = self.STATE_LOOK_FOR_HEADER

        self._thread = threading.Thread(target=self._rx_irq, args=(self._connected,))
        self._thread.daemon = True
        self._thread.start()

    def close(self):
        self._connected = False
        self._ser.close()  # If the while loop ends, we disconnected, so close serial port

    def readable(self):
        return self._readable

    def read(self):
        if self._readable:
            self._readable = False
            return self._rx_data

    def write(self, tx_data):
        if type(tx_data) is not self.Tx_type:
            raise Exception("Data is not of type " + str(self.Tx_type))

        self._ser.write(RoverSerial.transmit_header)
        self._ser.write(tx_data.toBytes())

    def _rx_irq(self, connected):
        while connected:
            while self._ser.readable():
                # Get byte from serial
                b = self._ser.read(1)

                if len(b) == 0:
                    continue

                # Look for the header
                if self._state == RoverSerial.STATE_LOOK_FOR_HEADER:
                    # shift history left
                    for i in range(len(RoverSerial.receive_header) - 1):
                        self._rx_header[i] = self._rx_header[i + 1]

                    # put new byte at the end of the array
                    self._rx_header[
                        len(RoverSerial.receive_header) - 1
                    ] = int.from_bytes(b, "little")

                    # if header is valid, move to read data state
                    # else continue looking for it
                    if self._rx_header == RoverSerial.receive_header:
                        self._state = RoverSerial.STATE_READING_DATA

                elif self._state == RoverSerial.STATE_READING_DATA:
                    # read bytes until we get to Rx_type
                    if len(self._rx_bytes) < self._rx_data.size:
                        self._rx_bytes.append(
                            int.from_bytes(b, "little")
                        )  # add byte to bytearray

                    if len(self._rx_bytes) == self._rx_data.size:
                        self._rx_data.fromBytes(self._rx_bytes)
                        self._readable = True

                        # reset state no matter what
                        self._state = RoverSerial.STATE_LOOK_FOR_HEADER
                        self._rx_bytes = bytearray()

                        if self._receive_cb:
                            self._receive_cb(self.read())


### ------------------- The code we care about is below this line ------------------- ###


# typedef struct {
#   int sensorPos;
#   float sensorFreq;
# } light_sensor_reading_t;
# class LightSensorReading(Struct):
#     def __init__(self):
#         self.pos = 0
#         self.freq = 0.0
#         super().__init__()

#     def __repr__(self):
#         return f"{self.pos}: {self.freq}"

#     def getPropertyList(self):
#         return ["pos", "freq"]


NUM_LIGHT_SENSORS = 6

# typedef struct {
#   float sensorFreq[NUM_LIGHT_SENSORS];
#   long int sensorTime[NUM_LIGHT_SENSORS];
#   // light_sensor_reading_t sensorReading; // response to most recent request, or null for none
#   float linActPos;  // float from 0-1 representing current pos of linear actuator
#   int pumpDir;      // -1 for "pump is moving backwards", 0 for pump not moving, 1 for forwards
# } ld_status_t;
class LDStatus(Struct):
    def __init__(self):
        self.sensor_freq = [0.0] * NUM_LIGHT_SENSORS
        self.sensor_time = [0] * NUM_LIGHT_SENSORS
        # self.sensor_reading = LightSensorReading()
        self.lin_act_pos = 0.0
        self.pump_dir = 0
        self.posn6recieved = 0
        self.lightReqRecieved = 0

        super().__init__()

    def __repr__(self):
        return f"LDStatus({self.sensor_freq}, {self.sensor_time}, {self.lin_act_pos}, {self.pump_dir}, {self.posn6recieved}, {self.lightReqRecieved})"

    def getPropertyList(self):
        return [
            "sensor_freq",
            "sensor_time",
            "lin_act_pos",
            "pump_dir",
            "posn6recieved",
            "lightReqRecieved",
        ]


# typedef struct {
#   float linActPos;         // float from 0-1 representing goal potentiometer reading
#   int pumpDir;             // -1 for backwards, 0 for off, 1 for forwards
#   int lightSensorRequest;  // int from 0-4 representing which light sensor we want a reading from, OR -1 to represent no request
#   float velocity;          // velocity
#   bool isPos;              // whether it's in position mode
# } ld_command_t;


class LDCommand(Struct):
    def __init__(self):
        self.lin_act_pos = 0.0
        self.pump_dir = 0
        self.light_sensor_request = -1
        self.light_sensor_request_id = -1
        self.velocity = 0.0
        self.isPos = True

        super().__init__()

    def __repr__(self):
        return f"LDCommand({self.lin_act_pos}, {self.pump_dir}, {self.light_sensor_request})"

    def getPropertyList(self):
        return [
            "lin_act_pos",
            "pump_dir",
            "light_sensor_request",
            "light_sensor_request_id",
            "velocity",
            "isPos",
        ]


class LDBoard:
    def __init__(self):
        # initialize state variables for command fields
        self.act_posn_cmd = 0.0
        self.pump_dir_cmd = 0
        self.light_sens_cmd = -1
        self.velocity_cmd = 0.0
        self.ctrl_mode_cmd = True

        # initialize LDCommand Struct object
        self.ld_command = LDCommand()

        # state variable for last act posn command
        self.send_cmd_again = True
        # position 6 stuff bcs that is weird
        self.sendingPosn6 = 0

        self.switch_back_to_posn = False
        self.last_act_posn = 0.0
        self.last_valid_act_posn = 0.0
        self.positions = [0.1, 0.23, 0.45, 0.71, 0.95, 1.2]

        # reset act cmd publisher
        self.act_set_cmd_pub = rospy.Publisher("act_posn_set", Int16, queue_size=10)

        # stuff for light sensor requests
        self.light_request_time = 0
        self.start_time = 0
        # light sensor publishers
        self.light_sens_pub = rospy.Publisher("light_sens_val", Float32, queue_size=10)
        self.light_time_pub = rospy.Publisher("light_sens_time", Float32, queue_size=10)
        # light sensor flag variable things
        self.last_light_request = -1
        self.light_req_ID = -1
        self.last_valid_light_cmd = -1
        self.LIGHT_TIME_WAIT = 3
        self.last_light_val = -1
        self.dummy_num_light = 0
        self.sendLightAgain = False

    # callback functions
    def ctrl_mode_callback(self, msg):
        if self.ctrl_mode_cmd == False and msg.data:
            self.switch_back_to_posn = True
        else:
            self.switch_back_to_posn = False
        self.ctrl_mode_cmd = msg.data

    def act_posn_callback(self, msg):
        if msg.data == 0:
            self.act_posn_cmd = 0.01
            self.sendingPosn6 = 0
        elif msg.data == 1:
            self.act_posn_cmd = 0.23
            self.sendingPosn6 = 0
        elif msg.data == 2:
            self.act_posn_cmd = 0.45
            self.sendingPosn6 = 0
        elif msg.data == 3:
            self.act_posn_cmd = 0.71
            self.sendingPosn6 = 0
        elif msg.data == 4:
            self.act_posn_cmd = 0.95
            self.sendingPosn6 = 0
        elif msg.data == 5:
            self.act_posn_cmd = 1.2
            self.sendingPosn6 = 1

    def act_vel_callback(self, msg):
        # just as a safeguard
        if self.ctrl_mode_cmd:
            # if in position control mode, velocity is 0
            self.velocity_cmd = 0.0
        else:
            # if in velocity mode, publish the velocity
            self.velocity_cmd = msg.data

    def light_sens_callback(self, msg):
        # grab the data (and set the data)
        self.light_sens_cmd = (
            msg.data
        )  # this is the command sent, set to -1 when command is recieved
        self.last_light_request = msg.data
        # need these bcs the light_sens_cmd used to construct commands is set back to -1 ^
        self.last_valid_light_cmd = msg.data

        self.light_req_ID += 1
        if self.start_time == 0:
            self.start_time = Time.now().secs
        self.light_request_time = Time.now().secs
        self.sendLightAgain = True

    def pump_callback(self, msg):
        self.pump_dir_cmd = msg.data

    def construct_cmd(self):
        self.ld_command.lin_act_pos = self.act_posn_cmd
        self.ld_command.pump_dir = self.pump_dir_cmd
        self.ld_command.light_sensor_request = self.light_sens_cmd
        self.ld_command.light_sensor_request_id = self.light_req_ID
        self.ld_command.velocity = self.velocity_cmd
        self.ld_command.isPos = self.ctrl_mode_cmd
        return self.ld_command

    def has_state_changed(self, newCommand):
        # can I just return the boolean logic?? ASK ABT THIS - NOTE
        if (
            self.last_state.lin_act_pos != newCommand.lin_act_pos
            or self.last_state.pump_dir != newCommand.pump_dir
            or self.last_state.light_sensor_request != newCommand.light_sensor_request
            or self.last_state.velocity != newCommand.velocity
            or self.last_state.isPos != newCommand.isPos
        ):
            return True
        else:
            return False

    def switch_back_posn(self, lin_act_posn):
        arr = np.asarray(self.positions)
        closestCmd = (np.abs(arr - lin_act_posn)).argmin()
        self.act_set_cmd_pub.publish(closestCmd)

    def reset_cmds(self):
        if self.last_light_request != -1:
            self.light_sens_cmd = -1
            self.light_req_ID += 1
            self.last_light_request = -1
            self.dummy_num_light = 0

    def set_last_state(self):
        self.last_state.lin_act_pos = self.act_posn_cmd
        self.last_state.pump_dir = self.pump_dir_cmd
        self.last_state.light_sensor_request = self.light_sens_cmd
        self.last_state.velocity = self.velocity_cmd
        self.last_state.isPos = self.ctrl_mode_cmd

    def publish_light_data(self, light_val):
        time_delta = Time.now().secs - self.light_request_time
        if self.last_valid_light_cmd >= 0 and self.last_valid_light_cmd < 6:
            if time_delta > (self.LIGHT_TIME_WAIT - 0.05) and time_delta < (
                self.LIGHT_TIME_WAIT + 0.05
            ):
                if light_val != self.last_light_val and self.dummy_num_light == 0:
                    self.light_sens_pub.publish(light_val)
                    self.dummy_num_light = 1
            # self.light_time_pub.publish(light_time)
        self.last_light_val = light_val


if __name__ == "__main__":
    # initialize stuff
    rospy.init_node("ld_board")
    ld_board = LDBoard()

    # create subscribers
    ctrl_mode_sub = rospy.Subscriber("ctrl_mode", Bool, ld_board.ctrl_mode_callback)
    act_posn_sub = rospy.Subscriber("act_posn", Int16, ld_board.act_posn_callback)
    act_vel_sub = rospy.Subscriber("velocity", Float32, ld_board.act_vel_callback)
    light_sens_sub = rospy.Subscriber(
        "light_sensor", Int16, ld_board.light_sens_callback
    )
    pump_sub = rospy.Subscriber("pump_cmd", Int16, ld_board.pump_callback)
    # create publishers
    act_posn_pub = rospy.Publisher("act_curr_posn", Float32, queue_size=10)

    # RoverSerial callback
    def receive_cb(status):
        # publish relevant information
        act_posn_pub.publish(status.lin_act_pos)
        ld_command = ld_board.construct_cmd()

        # check whether or not to publish light sensor data
        ld_board.publish_light_data(status.sensor_freq[ld_board.last_valid_light_cmd])

        # test if board has switched back to position mode recently, want to stay in same position if so
        if ld_board.switch_back_to_posn:
            if ld_command.lin_act_pos == ld_board.last_act_posn:
                ld_board.send_cmd_again = False
                ld_board.switch_back_posn(status.lin_act_pos)  # TESTING THIS RN!!!
                ld_board.act_posn_cmd = -1
            else:
                ld_board.switch_back_to_posn = False

        elif ld_board.ctrl_mode_cmd:
            if ld_board.sendingPosn6 == 1 and status.posn6recieved == 0:
                ld_board.send_cmd_again = True
            elif status.posn6recieved == 1 and ld_board.sendingPosn6 == 1:
                ld_board.sendingPosn6 = 0
                ld_board.send_cmd_again = False
                ld_board.act_posn_cmd = -1
            elif (
                ld_board.act_posn_cmd <= 1.0
                and ld_board.act_posn_cmd >= 0
                and abs(status.lin_act_pos - ld_board.act_posn_cmd) >= 0.01
            ):
                ld_board.send_cmd_again = True
            else:
                ld_board.send_cmd_again = False
        else:
            ld_board.send_cmd_again = True

        if status.lightReqRecieved:
            ld_board.sendLightAgain = False

    # initialize RoverSerial stuff
    serial_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF495157808667173330-if02"
    ser = RoverSerial(serial_port, 57600, LDCommand, LDStatus, receive_cb)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # send command
        ld_command = ld_board.construct_cmd()
        # test if the position command needs to be sent again
        if ld_board.send_cmd_again:
            ld_board.last_act_posn = ld_command.lin_act_pos
        else:
            ld_command.lin_act_pos = -1

        if not (ld_board.sendLightAgain):
            ld_board.reset_cmds()

        ser.write(ld_command)
        rate.sleep()

    ser.close()

