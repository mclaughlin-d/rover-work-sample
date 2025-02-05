import sys
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool


from roverserial import Struct, RoverSerial


class UvSpecStatus(Struct):
    def __init__(self):
        self.reading = 0.0
        super().__init__()

    def __repr__(self):
        return f"UvSpecStatus({self.reading})"

    def getPropertyList(self):
        return ["reading"]


class UvSpecCommand(Struct):
    def __init__(self):
        self.visible_led1 = False
        self.visible_led2 = False
        self.visible_led3 = False
        self.pump_on = False
        self.pump_forward = False

        super().__init__()

    def __repr__(self):
        return (
            f"UvSpecCommand({self.visible_led1},"
            f" {self.visible_led2}, {self.visible_led3}, {self.pump_on}, {self.pump_forward})"
        )

    def getPropertyList(self):
        return [
            "visible_led1",
            "visible_led2",
            "visible_led3",
            "pump_on",
            "pump_forward",
        ]


class UvSpecForwarder(Node):
    def __init__(self):
        super().__init__("uv_spec_forwarder")

        self.ld_cmd = UvSpecCommand()

        self.timer = self.create_timer(1, self.timer_callback)
        self.uv_spec_pub = self.create_publisher(Float32, "life_detection/spec_reading", 1)

        try:
            self.ser = RoverSerial(
                "/dev/serial/by-id/usb-STMicroelectronics"
                "_STM32_STLink_066DFF555187534867210631-if02",
                57600,
                UvSpecCommand,
                UvSpecStatus,
                self.receive_data,
            )
        except serial.SerialException as e:
            self.get_logger().error("UV Spec board serial port not found! exiting...")
            self.get_logger().error(f"Exception: {e}")
            sys.exit(1)

        self.pump_speed_sub = self.create_subscription(
            Int32, "life_detection/pump_dir", self.set_pump_dir, 1
        )
        self.visible1_led_sub = self.create_subscription(
            Bool, "life_detection/visible_led1", self.set_visible1, 1
        )

        self.visible2_led_sub = self.create_subscription(
            Bool, "life_detection/visible_led2", self.set_visible2, 1
        )

        self.visible3_led_sub = self.create_subscription(
            Bool, "life_detection/visible_led3", self.set_visible3, 1
        )

    def set_pump_dir(self, cmd: Int32):
        if cmd.data == 0:
            self.ld_cmd.pump_on = False
        elif cmd.data > 0:
            self.ld_cmd.pump_on = True
            self.ld_cmd.pump_forward = True
        elif cmd.data < 0:
            self.ld_cmd.pump_on = True
            self.ld_cmd.pump_forward = False

    def set_visible1(self, cmd: Bool):
        # self.get_logger().info(f'visible 1 {cmd}')
        self.ld_cmd.visible_led1 = cmd.data

    def set_visible2(self, cmd: Bool):
        # self.get_logger().info(f'visible 2 {cmd}')
        self.ld_cmd.visible_led2 = cmd.data

    def set_visible3(self, cmd: Bool):
        # self.get_logger().info(f'visible 3 {cmd}')
        self.ld_cmd.visible_led3 = cmd.data

    def receive_data(self, status: UvSpecStatus):
        # self.get_logger().info(f'status: {status}')
        self.uv_spec_pub.publish(Float32(data=status.reading * 3.05))

    def timer_callback(self):
        self.get_logger().info(f"writing {self.ld_cmd}")
        self.ser.write(self.ld_cmd)


def main(args=None):
    rclpy.init(args=args)
    pump_node = UvSpecForwarder()
    rclpy.spin(pump_node)
    pump_node.destroy_node()
    rclpy.shutdown()
