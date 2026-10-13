"""Files of the generated <robot>_kinematics package: EAIK all-solutions IK plus this app's FK."""
from pathlib import Path

from .moveit_config import hash_note, xml_note

HERE = Path(__file__).parent


def kinematics_files(robot: str, source: str) -> dict[str, str]:
    h, x, pkg = hash_note(source, robot), xml_note(source, robot), f"{robot}_kinematics"
    read = lambda name: (HERE / name).read_text(encoding="utf-8")
    return {
        f"resource/{pkg}": "",
        f"{pkg}/__init__.py": h + f'''"""Exact inverse kinematics for {robot}: every joint solution for a pose (EAIK), and FK."""
from .solver import Kinematics, fk, load, solve_all

__all__ = ["Kinematics", "fk", "load", "solve_all"]
''',
        f"{pkg}/_chain.py": h + read("chain.py"),
        f"{pkg}/_fk.py": h + read("fk.py").replace("from .chain import", "from ._chain import"),
        f"{pkg}/solver.py": h + read("templates/solver.py").replace("__ROBOT__", robot),
        "setup.py": h + f'''from setuptools import setup

package = "{pkg}"
setup(
    name=package,
    version="0.1.0",
    packages=[package],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package]),
                ("share/" + package, ["package.xml"])],
    install_requires=["setuptools"],
    zip_safe=True,
)
''',
        "package.xml": f"""<?xml version="1.0"?>
{x}<package format="3">
  <name>{pkg}</name>
  <version>0.1.0</version>
  <description>Exact inverse kinematics (all solutions) and FK for {robot}</description>
  <maintainer email="none@example.com">cad-to-urdf</maintainer>
  <license>BSD</license>
  <exec_depend>{robot}</exec_depend>
  <exec_depend>ament_index_python</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
""",
    }
