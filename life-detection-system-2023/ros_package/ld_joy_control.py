#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16, Float32, Bool
from sensor_msgs.msg import Joy
from abc import ABC, abstractmethod

"""
Relevant ROS nodes/topics:
    group name: "life_detection" (is prepended to topic names)
    node name: "ld_joy_control"
    joy subscriber: "life_detection/joy"
    publisher topic names
        ~~TO LD_BOARD.PY~~
        - actuator: "life_detection/act_posn/"
        - light sensors: "life_detection/light_sensor/" (maybe change to light_sens_cmd or smth)
        - pump: "life_detection/pump_cmd/"
        - command mode: "life_detection/ctrl_mode"
        - velocity: "life_detection/velocity"
"""


class LDControlType:
    POSITION_MODE = 0
    VELOCITY_MODE = 1


class XboxControllerState:
    def __init__(self, msg, constants, prior_state) -> None:
        self.prior_state = prior_state
        self.A = msg.buttons[constants["A"]]
        self.B = msg.buttons[constants["B"]]
        self.X = msg.buttons[constants["X"]]
        self.Y = msg.buttons[constants["Y"]]
        self.LB = msg.buttons[constants["LB"]]
        self.RB = msg.buttons[constants["RB"]]
        self.BACK = msg.buttons[constants["BACK"]]
        self.START = msg.buttons[constants["START"]]
        self.POWER = msg.buttons[constants["POWER"]]
        self.LEFT_STICK = msg.buttons[constants["LEFT_STICK"]]
        self.RIGHT_STICK = msg.buttons[constants["RIGHT_STICK"]]
        self.LEFT_STICK_HORIZONTAL = msg.axes[constants["LEFT_STICK_HORIZONTAL"]]
        self.LEFT_STICK_VERTICAL = msg.axes[constants["LEFT_STICK_VERTICAL"]]
        self.LEFT_TRIGGER = msg.axes[constants["LEFT_TRIGGER"]]
        self.RIGHT_STICK_HORIZONTAL = msg.axes[constants["RIGHT_STICK_HORIZONTAL"]]
        self.RIGHT_STICK_VERTICAL = msg.axes[constants["RIGHT_STICK_VERTICAL"]]
        self.RIGHT_TRIGGER = msg.axes[constants["RIGHT_TRIGGER"]]
        self.CROSS_HORIZONTAL = msg.axes[constants["CROSS_HORIZONTAL"]]
        self.CROSS_VERTICAL = msg.axes[constants["CROSS_VERTICAL"]]

    def has_changed(self, attr: str, call=False):
        if call:
            return (
                self.prior_state
                and getattr(self, attr)() != getattr(self.prior_state, attr)()
            )
        else:
            return self.prior_state and getattr(self, attr) != getattr(
                self.prior_state, attr
            )


class XboxController(ABC):
    def __init__(
        self, absolute_path, sub_path, state_class=XboxControllerState
    ) -> None:
        self.state_class = state_class
        self.constants = {
            "A": rospy.get_param(absolute_path + "/btnA"),
            "B": rospy.get_param(absolute_path + "/btnB"),
            "X": rospy.get_param(absolute_path + "/btnX"),
            "Y": rospy.get_param(absolute_path + "/btnY"),
            "LB": rospy.get_param(absolute_path + "/btnLB"),
            "RB": rospy.get_param(absolute_path + "/btnRB"),
            "BACK": rospy.get_param(absolute_path + "/btnBack"),
            "START": rospy.get_param(absolute_path + "/btnStart"),
            "POWER": rospy.get_param(absolute_path + "/btnPower"),
            "LEFT_STICK": rospy.get_param(absolute_path + "/btnLStick"),
            "RIGHT_STICK": rospy.get_param(absolute_path + "/btnRStick"),
            "LEFT_STICK_HORIZONTAL": rospy.get_param(
                absolute_path + "/axisLStickHoriz"
            ),
            "LEFT_STICK_VERTICAL": rospy.get_param(absolute_path + "/axisLStickVert"),
            "LEFT_TRIGGER": rospy.get_param(absolute_path + "/axisLT"),
            "RIGHT_STICK_HORIZONTAL": rospy.get_param(
                absolute_path + "/axisRStickHoriz"
            ),
            "RIGHT_STICK_VERTICAL": rospy.get_param(absolute_path + "/axisRStickVert"),
            "RIGHT_TRIGGER": rospy.get_param(absolute_path + "/axisRT"),
            "CROSS_HORIZONTAL": rospy.get_param(absolute_path + "/axisCrossHoriz"),
            "CROSS_VERTICAL": rospy.get_param(absolute_path + "/axisCrossVert"),
        }
        self.joy_sub = rospy.Subscriber(sub_path, Joy, self.update_internal)
        self.state = None

    @abstractmethod
    def update(self, msg):
        pass

    def update_internal(self, msg):
        self.state = self.state_class(msg, self.constants, self.state)
        self.update(self.state)


class LDXboxControllerState(XboxControllerState):
    def __init__(self, msg, constants, prior_state) -> None:
        super().__init__(msg, constants, prior_state)

    def stop(self):
        return self.CROSS_HORIZONTAL != 0 or self.CROSS_VERTICAL != 0


class LDXboxController(XboxController):
    # topic names
    JOY_TOPIC = "joy"
    ACT_POSN_TOPIC = "act_posn/"
    LIGHT_SENS_TOPIC = "light_sensor/"
    PUMP_TOPIC = "pump_cmd/"
    CONTROL_MODE_TOPIC = "ctrl_mode"
    VELOCITY_TOPIC = "velocity"

    def __init__(self, absolute_path, sub_path) -> None:

        # LD buttons - will change control scheme later
        # actuator position: A
        # light sensor number: Y
        # toggle pump: X
        # toggle control mode: B
        # velocity: right joystick (vertical)

        # initialize publishers
        self.actuator_posn_pub = rospy.Publisher(
            self.ACT_POSN_TOPIC, Int16, queue_size=10
        )
        self.light_sens_pub = rospy.Publisher(
            self.LIGHT_SENS_TOPIC, Int16, queue_size=10
        )
        self.pump_cmd_pub = rospy.Publisher(self.PUMP_TOPIC, Int16, queue_size=10)
        self.control_mode_pub = rospy.Publisher(
            self.CONTROL_MODE_TOPIC, Bool, queue_size=10
        )
        self.velocity_cmd_pub = rospy.Publisher(
            self.VELOCITY_TOPIC, Float32, queue_size=10
        )
        self.light_sens_num_pub = rospy.Publisher(
            "light_sens_num", Int16, queue_size=10
        )

        # initialize subscriber
        self.set_act_cmd_sub = rospy.Subscriber("act_posn_set", Int16, self.setActPosn)
        # initialize state variables for published messages
        self.actuator_posn_num = 0
        self.light_sens_num = 0
        self.pump_cmd_msg = Int16()
        self.pump_cmd_msg.data = 0
        self.vel_cmd_msg = Float32()
        self.vel_cmd_msg.data = 0
        self.control_mode_msg = True

        # initialize control mode type:
        self.control_mode = LDControlType.POSITION_MODE

        super().__init__(absolute_path, sub_path, LDXboxControllerState)

    def setActPosn(self, msg):
        self.actuator_posn_num = msg.data

    def update(self, state: LDXboxControllerState):
        # publish either position or velocity to actuator, depending on control mode
        if self.control_mode == LDControlType.POSITION_MODE:
            if state.has_changed("A") and self.state.A:
                self.actuator_posn_num = (self.actuator_posn_num + 1) % 6
                self.actuator_posn_pub.publish(Int16(self.actuator_posn_num))
        elif self.control_mode == LDControlType.VELOCITY_MODE:
            self.vel_cmd_msg.data = self.state.RIGHT_STICK_VERTICAL
            self.velocity_cmd_pub.publish(self.vel_cmd_msg)

        # toggle the control mode if 'B' is pressed
        if state.has_changed("B") and self.state.B:
            if self.control_mode == LDControlType.POSITION_MODE:
                self.control_mode = LDControlType.VELOCITY_MODE
                self.control_mode_msg = False
            else:
                self.control_mode = LDControlType.POSITION_MODE
                self.control_mode_msg = True
            self.control_mode_pub.publish(self.control_mode_msg)

        # toggle the light sensor, from 0-5, if 'X' is pressed
        if state.has_changed("Y") and self.state.Y:
            self.light_sens_num = (self.light_sens_num + 1) % 6
            self.light_sens_num_pub.publish(self.light_sens_num)
        if state.has_changed("RB") and self.state.RB:
            self.light_sens_pub.publish(Int16(self.light_sens_num))

        # toggles pump message and publishes
        if state.has_changed("X") and self.state.X:
            if self.pump_cmd_msg.data == 0:
                self.pump_cmd_msg.data = 1
            else:
                self.pump_cmd_msg.data = 0
            self.pump_cmd_pub.publish(self.pump_cmd_msg)
        # sets pump message to reverse
        if state.has_changed("LB") and self.state.LB:
            self.pump_cmd_msg.data = -1
            self.pump_cmd_pub.publish(self.pump_cmd_msg)


if __name__ == "__main__":
    rospy.init_node("ld_joy_control")

    ld_joy_control = LDXboxController("ld_joy_control", "/life_detection/joy")

    # publish which light sensor starts off as selected
    ld_joy_control.light_sens_num_pub.publish(ld_joy_control.light_sens_num)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        rate.sleep()

