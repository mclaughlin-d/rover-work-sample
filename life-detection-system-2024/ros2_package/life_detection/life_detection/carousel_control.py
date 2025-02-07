#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler

from std_msgs.msg import Float32

class DynamixelConfig:
    def __init__(self, id, name, current_limit, pwm_limit):
        self.id = id
        self.name = name
        self.current_limit = current_limit
        self.pwm_limit = pwm_limit


class CarouselDynamixel:
    class ControlTable:
        ADDR_TORQUE_ENABLE = 64
        ADDR_GOAL_POSITION = 116
        ADDR_GOAL_CURRENT = 102
        ADDR_GOAL_VELOCITY = 104
        ADDR_PRESENT_POSITION = 132
        ADDR_PRESENT_VELOCITY = 128
        ADDR_PRESENT_CURRENT = 126
        ADDR_OPERATING_MODE = 11
        ADDR_CURRENT_LIMIT = 38
        ADDR_PWM_LIMIT = 36
        ADDR_HOMING_OFFSET = 20

    class OperatingMode:
        CURRENT_CONTROL_MODE = 0
        VELOCITY_CONTROL_MODE = 1
        POSITION_CONTROL_MODE = 3
        EXT_POSITION_CONTROL_MODE = 4
        CURRENT_BASED_POSITION_CONTROL_MODE = 5
        PWM_CONTROL_MODE = 16

    def __init__(
        self,
        id: int,
        device_name: str,
        current_limit: int,
        pwm_limit: int,
        baud_rate: int,
    ):
        self.config = DynamixelConfig(id, "carousel", current_limit, pwm_limit)

        self.DEGREES_PER_PULSE = 0.088 / 3

        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(2.0)  # No support for 1.0!

        # Open port
        self.portHandler.openPort()
        self.portHandler.setBaudRate(baud_rate)

        # disable torque
        self._write(self.ControlTable.ADDR_TORQUE_ENABLE, False, 1)
        # set the operating mode
        self._write(
            self.ControlTable.ADDR_OPERATING_MODE,
            self.OperatingMode.CURRENT_BASED_POSITION_CONTROL_MODE,
            1,
        )
        # re-enable torque
        self._write(self.ControlTable.ADDR_TORQUE_ENABLE, True, 1)

    def _degrees_to_pulses(self, degrees):
        return int(degrees / self.DEGREES_PER_PULSE)

    def _pulses_to_degrees(self, pulses):
        return int(pulses) * self.DEGREES_PER_PULSE

    def _write(self, addr, data, len: int) -> None:
        if len == 1:
            self.packetHandler.write1ByteTxRx(self.portHandler, self.config.id, addr, data)
        elif len == 4:
            self.packetHandler.write4ByteTxRx(self.portHandler, self.config.id, addr, data)

    def _read(self, addr, len):
        if len == 1:
            data, result, error = self.packetHandler.read1ByteTxRx(
                self.portHandler, self.config.id, addr
            )
        if len == 4:
            data, result, error = self.packetHandler.read4ByteTxRx(
                self.portHandler, self.config.id, addr
            )
        else:
            raise ValueError("Unsupported message length!")
        return data, result, error

    def read_position(self):
        posn_data, result, error = self._read(self.ControlTable.ADDR_PRESENT_POSITION, 4)
        return self._pulses_to_degrees(posn_data), result, error

    def write_position(self, posn: float) -> None:
        raw_posn = self._degrees_to_pulses(posn)
        self._write(self.ControlTable.ADDR_GOAL_POSITION, raw_posn, 4)


class CarouselNode(Node):
    def __init__(self, carousel_dyna: CarouselDynamixel):
        super().__init__("carousel_control")

        self.dyna = carousel_dyna

        self.posn_sub = self.create_subscription(
            Float32, "/life_detection/goal_carousel_posn", self.move_carousel, 1
        )
        self.posn_pub = self.create_publisher(Float32, "/life_detection/curr_carousel_posn", 1)

        self.create_timer(0.1, self.publish_posn)

    def move_carousel(self, msg: Float32):
        self.dyna.write_position(msg.data)

    def publish_posn(self):
        data, result, error = self.dyna.read_position()
        if result != 0:
            self.get_logger().error(f"Error: {error} occured when reading from dynamixel!")
        else:
            self.posn_pub.publish(Float32(data=data))


def main(args=None):
    rclpy.init(args=args)
    dynamixel = CarouselDynamixel(
        19,
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3WFF84-if00-port0",
        400,
        300,
        1000000,
    )
    node = CarouselNode(dynamixel)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
