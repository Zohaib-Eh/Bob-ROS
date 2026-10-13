"""Container side of run_task.sh: reset the simulation, run one program, print RESULT."""
import runpy
import sys
import traceback

from robot import Robot, RobotError


def main(path: str) -> int:
    try:
        Robot().reset()
    except Exception as exc:  # the session itself is broken; nothing the program can fix
        print(f"RESULT: failed: could not reset the simulation ({exc})", flush=True)
        return 1
    try:
        runpy.run_path(path, run_name="__main__")
    except RobotError as exc:
        print(f"RESULT: failed: {type(exc).__name__}: {exc}", flush=True)
        return 1
    except SystemExit as exc:
        if exc.code not in (None, 0):
            print(f"RESULT: failed: the program exited with {exc.code}", flush=True)
            return 1
    except Exception as exc:
        traceback.print_exc()
        print(f"RESULT: failed: {type(exc).__name__}: {exc}", flush=True)
        return 1
    print("RESULT: ok", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
