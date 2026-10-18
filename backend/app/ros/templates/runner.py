"""Container side of run_task.sh: reset the simulation, run one program (recorded for replay), print
TRACE then RESULT."""
import runpy
import sys
import traceback

import recorder
import scene
import sim_ctl
from robot import Robot, RobotError


def _run(path: str) -> str:
    """The program's outcome, as the text after 'RESULT: '."""
    try:
        runpy.run_path(path, run_name="__main__")
    except RobotError as exc:
        return f"failed: {type(exc).__name__}: {exc}"
    except SystemExit as exc:
        if exc.code not in (None, 0):
            return f"failed: the program exited with {exc.code}"
    except Exception as exc:
        traceback.print_exc()
        return f"failed: {type(exc).__name__}: {exc}"
    return "ok"


def main(path: str) -> int:
    try:
        sim_ctl.prepare(path)  # the world this program declared: workcell + SCENE (MuJoCo or MoveIt only)
        robot = Robot()
        robot.reset()
    except scene.SceneError as exc:  # the program's objects are wrong; the program can fix that
        print(f"RESULT: failed: {exc}", flush=True)
        return 1
    except Exception as exc:  # the session itself is broken; nothing the program can fix
        print(f"RESULT: failed: could not reset the simulation ({exc})", flush=True)
        return 1
    rec = recorder.begin(path, robot)   # never raises; failed runs are recorded too
    result = _run(path)
    print(rec.end(result), flush=True)
    print(f"RESULT: {result}", flush=True)
    return 0 if result == "ok" else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
