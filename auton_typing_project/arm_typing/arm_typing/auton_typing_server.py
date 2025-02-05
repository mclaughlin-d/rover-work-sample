from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import String

from arm_typing.character_typer import CharacterTyper

from arm_msgs.msg import ControlMode
from arm_msgs.srv import ModeRequest


class AutonTypingServer:
    def __init__(self, node: Node, config_path: str) -> None:
        self.node = node
        self.config_path = config_path
        self.character_typer: CharacterTyper = CharacterTyper(self.config_path, node)
        self.phrases: List[str] = []

        # mode client for switching to gripper IK mode
        self.mode_client = self.node.create_client(ModeRequest, "/arm/mode/request")
        self.responses: List[Tuple[Future, ControlMode]] = []

        # subscriber for listening to phrases to load into server
        self.phrase_subscriber = self.node.create_subscription(
            String, "/arm/typing/phrase", self.load_phrase, 1
        )

        # subscribe for listening to switch into typing
        self.typing_subscriber = self.node.create_subscription(
            String, "/arm/typing/start", self.execute_phrase, 1
        )

    def load_phrase(self, phrase: String) -> None:
        if not self.character_typer.is_phrase_supported(phrase.data):
            self.node.get_logger().error("Phrase is not supported by keyboard!")
            return
        if phrase.data in self.phrases:
            self.node.get_logger().error("Phrase is already loaded!")
            return

        self.phrases.append(phrase.data)

    def execute_phrase(self, phrase: String) -> None:
        # get phrase
        if phrase.data not in self.phrases:
            self.node.get_logger().error("Phrase is not loaded!")
            return

        # switch arm to Gripper IK if not already in that mode
        if not self.request():
            self.node.get_logger().error("Unable to switch to gripper IK mode!")
            return

        for c in phrase.data:
            self.character_typer.type_character(c)

        pass

    def request(self) -> bool:
        if not self.mode_client.service_is_ready():
            self.node.get_logger().error("Control mode request failed!")
            return False

        req = ModeRequest.Request(
            mode=ControlMode(mode=ControlMode.MODE_IK_GRIPPER_FRAME)
        )
        self.responses.append((self.mode_client.call_async(req), req.mode))
        return True


class AutonTypingNode(Node):
    def __init__(self) -> None:
        super().__init__("auton_typing_server")

        self.declare_parameter("keyboard_config_path", "")
        self.config_path = self.get_parameter("keyboard_config_path").value

        self.auton_typing_server = AutonTypingServer(
            node=self, config_path=self.config_path
        )


def main(args=None):
    rclpy.init(args=args)
    auton_typing_node = AutonTypingNode()
    rclpy.spin(auton_typing_node)
    auton_typing_node.destroy_node()
    rclpy.shutdown()
