# ROS py import, basically always required
import rclpy 
# Node import, needed to create ROS Nodes
from rclpy.node import Node 

# import the message type
from std_msgs.msg import Float32

# create a class for this Node which inherits from Node class
class MotorDegreePublisher(Node):
    # create some static variables
    TIMER_PERIOD = 0.1
    DEGREE_INCR = 0.1
    
    def __init__(self):
        # call parent init function, pass name of node
        # this name will show up when you run `ros2 node list`
        super().__init__('motor_degree_publisher')
        
        # create a publisher
        self.degree_publisher = self.create_publisher(Float32, 'motor_degree_cmd', 1)

        # create a timer to update the command message and publish
        self.timer = self.create_timer(self.TIMER_PERIOD, self.update_degree_cmd)

        # create a variable to store the current degree command
        self.degree_cmd = Float32() # use the ROS message type that will be published

    def update_degree_cmd(self) -> None:
        """Is called every TIMER_PERIOD seconds to update deg cmd value and publish
        """
        # Don't forget to access the 'data' field of the Float32 message!
        self.degree_cmd.data = (self.degree_cmd.data + self.DEGREE_INCR) % 360
        self.degree_publisher.publish(self.degree_cmd)


# add a main method
def main(args=None):
    # init  ROS
    rclpy.init(args=args)

    motor_degree_publisher = MotorDegreePublisher()

    # get things started
    rclpy.spin(motor_degree_publisher)
