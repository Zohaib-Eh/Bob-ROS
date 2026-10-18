"""Run replay: the trace runner.py records for every run, <robot>_tasks/runs/<program>-<UTC>.json, read by robot
name from the workspace on disk (so replays work with the session stopped)."""
import json
import re
from pathlib import Path

from app.program.session import WORKSPACES
from app.ros.moveit_config import ROS_NAME

TRACE_LINE = re.compile(r"^TRACE: (runs/[A-Za-z0-9_.-]+\.json)\s*$", re.M)
NAME = re.compile(r"^[A-Za-z0-9_.-]+\.json$")


def trace_of(output: str) -> str | None:
    """The trace a run printed ('TRACE: runs/…json'); None when it printed 'TRACE: none (…)' or nothing."""
    found = TRACE_LINE.findall(output or "")
    return found[-1] if found else None


def read(robot: str, name: str) -> dict:
    """ValueError for a bad robot or file name, FileNotFoundError when there is no such trace."""
    if not ROS_NAME.match(robot or "") or not NAME.match(name or "") or ".." in name:
        raise ValueError("Not a trace name.")
    return json.loads((Path(WORKSPACES) / robot / "src" / f"{robot}_tasks" / "runs" / name).read_text(encoding="utf-8"))
