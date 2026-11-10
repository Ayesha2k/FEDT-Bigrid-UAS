"""Single runtime configuration; robot IDs come from the editable Webots world."""
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parents[1]
RUNTIME = HERE / ".runtime"
OWNER = "cps.asset.crazyflie.webots3"


def drone_ids(world):
    # Remove comments and quoted strings before balancing VRML node braces.
    text = re.sub(r'"(?:\\.|[^"\\])*"|#[^\n]*', '', Path(world).read_text())
    ids = []
    for match in re.finditer(r'\bCrazyflie\s*\{', text):
        depth, end = 1, match.end()
        while depth and end < len(text):
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        if depth:
            raise ValueError("Unclosed Crazyflie node in world")
        field = re.search(r'\bid\s+(-?\d+)\b', text[match.end():end-1])
        if not field:
            raise ValueError("Each Crazyflie node must have an explicit id")
        value = int(field[1])
        if not 1 <= value <= 255 or value in ids:
            raise ValueError(f"Invalid or duplicate Crazyflie id: {value}")
        ids.append(value)
    if not ids:
        raise ValueError("World contains no Crazyflie nodes")
    return sorted(ids)


def load(world=None):
    config = json.loads((HERE / "config.json").read_text())
    config["drone_ids"] = drone_ids(world or HERE.parent / "crazyflie.wbt")
    return config


def ports(config):
    return [config[k] + i for k in ("http_port", "web_port", "ws_port")
            for i in range(len(config["drone_ids"]))]
