"""Generate a ROS 2 Jazzy workspace from a cad-to-urdf export.

Usage: python -m app.ros.cli <export.zip> -o <workspace dir> [--samples N]"""
import argparse
import re
import sys
import tempfile
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path

from .moveit_config import generate


def _robot_name(z: zipfile.ZipFile) -> str:
    """The export holds <robot>/urdf/<robot>.urdf."""
    found = [n.split("/")[0] for n in z.namelist() if re.fullmatch(r"(?P<r>[^/]+)/urdf/(?P=r)\.urdf", n)]
    if len(found) != 1:
        raise ValueError("Not a cad-to-urdf export.")
    return found[0]


def workspace_from_zip(zip_source: Path | bytes, out_dir: Path, samples: int = 10000) -> str:
    """Generate <out_dir>/src from a cad-to-urdf export zip (path or bytes). Returns the robot name."""
    import io
    src = io.BytesIO(zip_source) if isinstance(zip_source, bytes) else zip_source
    with zipfile.ZipFile(src) as z, tempfile.TemporaryDirectory() as tmp:
        robot = _robot_name(z)
        z.extractall(tmp)
        name = "upload.zip" if isinstance(zip_source, bytes) else Path(zip_source).name
        generate(robot, Path(tmp) / robot, Path(out_dir) / "src", source=name, samples=samples)
    return robot


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="python -m app.ros.cli")
    p.add_argument("zip")
    p.add_argument("-o", "--out", required=True, help="workspace dir; packages go in <out>/src")
    p.add_argument("--samples", type=int, default=10000, help="random poses for the collision matrix")
    a = p.parse_args(argv)
    src = Path(a.out) / "src"
    try:
        robot = workspace_from_zip(Path(a.zip), Path(a.out), a.samples)
    except (zipfile.BadZipFile, RuntimeError, NotImplementedError, RecursionError):
        print("error: Not a cad-to-urdf export.", file=sys.stderr)
        return 1
    except ET.ParseError:
        print("error: The URDF is not valid XML.", file=sys.stderr)
        return 1
    except (ValueError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {src}: {robot}, {robot}_moveit_config, {robot}_kinematics")
    return 0


if __name__ == "__main__":
    sys.exit(main())
