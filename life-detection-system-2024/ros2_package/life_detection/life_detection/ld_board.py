import sys
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from roverserial import Struct, RoverSerial


NUM_LIGHT_SENSORS = 6


class LDStatus(Struct):
    def __init__(self):
        self.rh_reading = 0.0
        self.f_temp = 0.0
        self.c_temp = 0.0
        super().__init__()

    def __repr__(self):
        return "LDStatus()"

    def getPropertyList(self):
        return ["rh_reading", "f_temp", "c_temp"]


class LDCommand(Struct):
    def __init__(self):
        self.drill_spin = 0.0
        self.stage_move = 0.0
        self.lin_actuator = 0.0
        self.pump_dir = 0
        self.lid_servo = 0.0

        super().__init__()

    def __repr__(self):
        return f"LDCommand({self.pump_dir})"

    def getPropertyList(self):
        return ["drill_spin", "stage_move", "lin_actuator", "pump_dir", "lid_servo"]


class PumpClass(Node):
    LID_SERVO_CLOSE = 0.048
    LID_SERVO_OPEN = 0.091
    LID_SERVO_STOP = 0.0

    def __init__(self):
        super().__init__("ld_board")

        self.pump_dir = 0

        self.ld_cmd = LDCommand()

        self.timer = self.create_timer(1, self.timer_callback)

        self.drill_spin = 0.0

        self.stage_move = 0.0

        self.lin_actuator = 0.0

        self.lid_servo = 0.0

        try:
            self.ser = RoverSerial(
                "/dev/serial/by-id/usb-STMicroelectronics"
                "_STM32_STLink_0671FF495257808667044219-if02",
                57600,
                LDCommand,
                LDStatus,
                self.receive_data,
            )
        except serial.SerialException as e:
            self.get_logger().error("Pump board serial port not found! exiting...")
            self.get_logger().error(f"Exception: {e}")
            sys.exit(1)

        self.pump_dir_sub = self.create_subscription(
            Int32, "life_detection/pump_dir", self.set_pump_dir, 1
        )
        self.drill_spin_sub = self.create_subscription(
            Float32, "life_detection/drill_spin", self.set_drill_spin, 1
        )
        self.stage_move_sub = self.create_subscription(
            Float32, "life_detection/stage_move", self.set_stage_move, 1
        )
        self.lin_actuator_sub = self.create_subscription(
            Float32, "life_detection/lin_actuator", self.set_lin_actuator, 1
        )
        self.lid_servo_sub = self.create_subscription(
            Int32, "life_detection/lid_servo", self.set_lid_servo, 1
        )
        self.lid_servo_direct = self.create_subscription(
            Float32, "/life_detection/lid_servo_direct", self.set_lid_servo_direct, 1
        )

        # publisehrs
        self.rh_pub = self.create_publisher(Float32, "life_detection/rh_reading", 1)
        self.f_temp_pub = self.create_publisher(Float32, "life_detection/f_temp", 1)
        self.c_temp_pub = self.create_publisher(Float32, "life_detection/c_temp", 1)

    def set_pump_dir(self, cmd: Int32):
        self.pump_dir = cmd.data
        self.ld_cmd.pump_dir = self.pump_dir

    def set_drill_spin(self, cmd: Float32):
        self.drill_spin = cmd.data
        self.ld_cmd.drill_spin = self.drill_spin

    def set_stage_move(self, cmd: Float32):
        self.stage_move = cmd.data
        self.ld_cmd.stage_move = self.stage_move

    def set_lin_actuator(self, cmd: Float32):
        self.lin_actuator = cmd.data
        self.ld_cmd.lin_actuator = self.lin_actuator

    def set_lid_servo(self, cmd: Int32):
        if cmd.data == 1:
            self.ld_cmd.lid_servo = self.LID_SERVO_OPEN
        else:
            self.ld_cmd.lid_servo = self.LID_SERVO_CLOSE

    def set_lid_servo_direct(self, cmd: Float32):
        self.ld_cmd.lid_servo = cmd.data

    def receive_data(self, status: LDStatus):
        self.rh_pub.publish(Float32(data=status.rh_reading))
        self.f_temp_pub.publish(Float32(data=status.f_temp))
        self.c_temp_pub.publish(Float32(data=status.c_temp))

    def timer_callback(self):
        self.ser.write(self.ld_cmd)


def main(args=None):
    rclpy.init(args=args)
    pump_node = PumpClass()
    rclpy.spin(pump_node)
    pump_node.destroy_node()
    rclpy.shutdown()
