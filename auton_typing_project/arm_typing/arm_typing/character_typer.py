from typing import List, Optional

from rclpy.node import Node

from arm_msgs.msg import IKInput, ArmCommand, ArmStatus
from arm_typing.config_parser import KeyboardConfig, parse_config

import asyncio
import time


class CharacterTyper:

    def __init__(self, config_path: str, node: Node) -> None:
        self.keyboard_configs: List[KeyboardConfig] = parse_config(config_path)
        self.keyboard_config = self.keyboard_configs[0]

        self.node = node

        self.typing_jobs: List[
            asyncio.Task
        ] = []  # IDEA - store typing jobs in list? some kind of queue
        self.current_job: Optional[asyncio.Task] = None

        # monitor current arm IK command and arm status
        self.last_ik_cmd = ArmCommand()
        self.curr_arm_posn = ArmStatus()

        # create publisher to mouse topic
        self.mouse_pub = self.node.create_publisher(
            IKInput, "/arm/ik_input/filtered", 10
        )

        # create a subscriber to the IK command topic
        self.ik_cmd_sub = self.node.create_subscription(
            ArmCommand, "/arm/command/ik", self.update_ik_cmd, 10
        )
        # create a subscriber to the current arm status
        self.arm_status_sub = self.node.create_subscription(
            ArmStatus, "/arm/status/all", self.update_arm_status, 10
        )

    def update_ik_cmd(self, cmd: ArmCommand) -> None:
        # TODO - need to make sure this is the command that correlates to IKInput msg
        self.last_ik_cmd = cmd

    def update_arm_status(self, msg: ArmStatus) -> None:
        self.curr_arm_posn = msg

    def at_character(self) -> bool:
        # TODO
        return True

    # TODO - make async?
    def type_character(self, character: str) -> None:
        try:
            character_config = self.keyboard_config.char_configs[character]
            # TODO - remove, just so variable is used
            self.node.get_logger().error(
                f"character config for char: {character_config.character}"
            )
        except KeyError:
            self.node.get_logger().error("Character not found in config!")
            return

        # for now, use parsed configs as the xyz delta and create a spacemouse w/ it
        mouse = IKInput()
        mouse.x = character_config.x_delta / 100.0
        mouse.y = character_config.y_delta / 100.0
        mouse.z = character_config.z_delta / 100.0

        # send goal position command
        self.mouse_pub.publish(mouse)

        # monitor position
        self.node.get_logger().error(
            f"Moving into position {mouse.x}, {mouse.y}, {mouse.z} to type"
            f"character {character_config.character}..."
        )

        # TODO - add some timeout, don't wait forever that's not a good idea
        while not self.at_character():
            continue

        # TODO - wait for arm to reach desired position instead of sleeping
        self.node.get_logger().error("Typing character!")
        time.sleep(1.0)
        self.node.get_logger().error("Typed character!")

        # move back to known character
        mouse.x = -character_config.x_delta
        mouse.y = -character_config.y_delta
        mouse.z = -character_config.z_delta

        self.mouse_pub.publish(mouse)

    def cancel_typing(self) -> None:
        pass

    def is_phrase_supported(self, phrase: str) -> bool:
        for c in phrase.lower():
            if c not in self.keyboard_config.char_configs.keys():
                return False
        return True
