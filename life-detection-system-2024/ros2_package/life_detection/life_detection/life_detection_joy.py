import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32, Bool

from controller_state import XboxControllerState, XboxController, deadzone


class LifeDetectionXBoxController(XboxController):
    # carousel modes
    CAROUSEL_PRESET_MODE = True
    CAROUSEL_VEL_MODE = False

    # carousel presets (by index)
    CAROUSEL_PRESETS = [60.0, 120.0, 180.0]

    # carousel position increment
    CAROUSEL_POSITION_INCR = 0.1 

    DRILL_SPEED = -0.5

    STAGE_MULT = 1.0

    def __init__(self, node: Node, sub_path: str) -> None:
        super().__init__(node, sub_path)

        # pump
        self.pump_dir = 1  # start with pump off
        self.pump_on = False

        # carousel
        self.carousel_curr_posn = 0.0
        self.carousel_last_posn_cmd = 0.0
        self.carousel_preset = 0
        self.carousel_cmd_mode = self.CAROUSEL_VEL_MODE

        # auger stage 1
        self.stage_1_cmd = 0

        # auger stage 2
        self.drill_cmd = 0  # Off?

        # auger stage 3
        self.stage_3_cmd = 0

        # UV spec
        self.uv_on = False

        # UV spec reading request
        self.last_read = 0.0

        self.lid_servo_open = False

        # create subscribers
        node.create_subscription(
            Float32, "/life_detection/curr_carousel_posn", self.carousel_posn_cb, 1
        )
        # create publishers
        self.carousel_pub = node.create_publisher(Float32, "/life_detection/goal_carousel_posn", 1)
        self.pump_pub = node.create_publisher(Int32, "/life_detection/pump_dir", 1)
        self.stage_1_pub = node.create_publisher(Float32, "/life_detection/stage_move", 1)
        self.stage_2_pub = node.create_publisher(Float32, "/life_detection/drill_spin", 1)
        self.stage_3_pub = node.create_publisher(Float32, "/life_detection/lin_actuator", 1)
        self.uv_pub = node.create_publisher(Bool, "/life_detection/uv", 1)
        self.lid_servo_pub = node.create_publisher(Int32, "/life_detection/lid_servo", 1)

        # TODO remove
        self.node = node

    def carousel_posn_cb(self, msg) -> None:
        self.carousel_curr_posn = msg.data

    def update_posn_cmd_carousel(self) -> None:
        if self.carousel_cmd_mode == self.CAROUSEL_VEL_MODE:
            self.carousel_last_posn_cmd = self.carousel_curr_posn

    def update(self, state: XboxControllerState) -> None:
        # pump direction
        if state.RB and state.has_changed("RB"):
            if self.pump_dir == 1:
                self.pump_dir = -1
            else:
                self.pump_dir = 1

        # pump on/off
        if state.START and state.has_changed("START"):
            self.pump_on = not self.pump_on

        if self.pump_on:
            self.pump_pub.publish(Int32(data=self.pump_dir))
        else:
            self.pump_pub.publish(Int32(data=0))

        # toggle carousel control mode
        if state.X and state.has_changed("X"):
            self.carousel_cmd_mode = not self.carousel_cmd_mode
            self.update_posn_cmd_carousel()
        if (
            self.carousel_cmd_mode == self.CAROUSEL_PRESET_MODE
            and state.Y
            and state.has_changed("Y")
        ):
            self.carousel_preset = (self.carousel_preset + 1) % 3
            self.carousel_pub.publish(Float32(data=self.CAROUSEL_PRESETS[self.carousel_preset]))
        elif self.carousel_cmd_mode == self.CAROUSEL_VEL_MODE and state.Y:
            self.carousel_last_posn_cmd += self.CAROUSEL_POSITION_INCR
            self.carousel_pub.publish(Float32(data=self.carousel_last_posn_cmd))
            # TODO ^ this is not variable speed, allocate a joystick instead???
            # only need one joystick for auger

        # auger stage 1
        self.stage_1_cmd = deadzone(state.LEFT_STICK_VERTICAL) * self.STAGE_MULT
        self.stage_1_pub.publish(Float32(data=self.stage_1_cmd))

        # auger stage 2 (drill)
        if state.LEFT_TRIGGER > 0:
            self.stage_2_pub.publish(Float32(data=self.DRILL_SPEED * state.LEFT_TRIGGER))
        else:
            self.stage_2_pub.publish(Float32(data=0.0))

        # auger stage 3
        self.stage_3_cmd = deadzone(state.RIGHT_STICK_VERTICAL) * self.STAGE_MULT
        self.stage_3_pub.publish(Float32(data=self.stage_3_cmd))

        # UV spectrometer buttons
        if state.A and state.has_changed("A"):
            self.uv_on = not self.uv_on
            self.uv_pub.publish(Bool(data=self.uv_on))

        if state.BACK and state.has_changed("BACK"):
            self.lid_servo_open = not self.lid_servo_open
            if self.lid_servo_open:
                self.lid_servo_pub.publish(Int32(data=1))
            else:
                self.lid_servo_pub.publish(Int32(data=-1))


class LDJoyControl(Node):
    def __init__(self):
        super().__init__("life_detection_joy")
        self.xbox = LifeDetectionXBoxController(self, "/life_detection/joy")


def main(args=None):
    rclpy.init(args=args)
    ld_joy_control = LDJoyControl()
    rclpy.spin(ld_joy_control)
    ld_joy_control.destroy_node()
    rclpy.shutdown()
