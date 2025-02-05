# ROS py import, basically always required
import rclpy 
# Node import, needed to create ROS Nodes
from rclpy.node import Node 

# import the message type
from std_msgs.msg import Float32

# import the custom motor state message type
from demo_msgs.msg import MotorState

# The 'motor' is represented by this class
class MockMotor():
    def __init__(self):
        self.angle_pos = 0.0
        self.torque = 0.0

    def move(self, degree: float) -> None:
        self.angle_pos = degree

    def read_pos(self) -> float:
        return self.angle_pos

    def read_torque(self) -> float:
        return self.torque


# create a class for this Node which inherits from Node class
class MotorController(Node):
    def __init__(self):
        # Step 1. Subscriber for
        super().__init__('motor_controller')
        self.cmd_sub = self.create_subscription(
            Float32,
            'motor_degree_cmd',
            self.handle_cmd,
            1
        )

        # Create an instance of the 'motor' to move
        self.motor = MockMotor()

        # Step 2. Custom message for motor state
        # NOTE - the custom message package creation will be more copy-paste for time,
        # this is the only stuff that's important to actually 
        self.motor_pub = self.create_publisher(MotorState, 'motor_state', 1)
        self.motor_state = MotorState()

    def handle_cmd(self, cmd: Float32) -> None:
        """Moves the 'motor' to the specified command and publishes state."""
        # log the info so we can see it!
        self.get_logger().info(f'Moving motor to {cmd.data} degrees')
        # call the motor move method
        self.motor.move(cmd.data)

        # get the motor state
        self.update_motor_state()
        # publish the motor state - TODO determine if this is better demonstrated as 
        # a timer callback idk
        self.motor_pub.publish(self.motor_state)

    def update_motor_state(self):
        """Updates the stored state of the 'motor' controlled by this class."""
        # set the fields of the MotorState message
        self.motor_state.angle = self.motor.read_pos()
        self.motor_state.torque = self.motor.read_torque()



def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)