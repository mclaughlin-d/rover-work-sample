from dataclasses import dataclass

import yaml
import string


@dataclass
class CharacterConfig:
    character: str
    x_delta: float
    y_delta: float
    z_delta: float


class KeyboardConfig:
    def __init__(self) -> None:
        self.name: str = ""
        self.known_key: str = ""
        self.char_configs: dict[str, CharacterConfig] = {}


def parse_config(config_path: str) -> list[KeyboardConfig]:
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    keyboard_configs: list[KeyboardConfig] = []
    for name, keyboard in config["keyboard"].items():
        keyboard_config = KeyboardConfig()
        keyboard_config.name = name
        keyboard_config.known_key = keyboard["known_key"]

        keyboard_deltas = keyboard["character_deltas"]

        # populate characters
        for c in string.ascii_lowercase:
            char_config = CharacterConfig(
                c,
                keyboard_deltas[c]["x_delta"],
                keyboard_deltas[c]["y_delta"],
                keyboard_deltas[c]["z_delta"],
            )
            keyboard_config.char_configs[c] = char_config

        # add to list
        keyboard_configs.append(keyboard_config)

    return keyboard_configs
