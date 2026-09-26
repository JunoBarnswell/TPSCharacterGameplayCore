"""Server-owned locomotion cycle tuning shared with the browser presentation."""

import json
from importlib.resources import files

from aster_game.game.movement.state import Gait

_DATA = json.loads(
    files("aster_game").joinpath("web/motion/locomotion-tuning.json").read_text(encoding="utf-8")
)
if _DATA["schema_version"] != 1 or _DATA["phase_owner"] != "server":
    raise ValueError("Unsupported locomotion tuning schema")

GAIT_STRIDE_LENGTHS = {
    Gait.WALK: _DATA["gaits"]["walk"]["stride_length"],
    Gait.RUN: _DATA["gaits"]["run"]["stride_length"],
    Gait.SPRINT: _DATA["gaits"]["sprint"]["stride_length"],
}
