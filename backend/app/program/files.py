"""The robot's ROS workspace as seen by the Program workspace's Code tab: list and read text files,
save only the user's programs."""
import re
from pathlib import Path, PurePosixPath

TEXT_EXT = {".py", ".yaml", ".yml", ".xml", ".xacro", ".srdf", ".urdf", ".md", ".sh", ".txt", ".cfg", ".json"}
SKIP_DIRS = {"build", "install", "log", "__pycache__", ".git"}


def safe(src, rel: str) -> Path:
    """Resolve a workspace-relative path; anything that could leave the workspace is refused."""
    if not rel or "\\" in rel or rel.startswith("/") or re.match(r"^[A-Za-z]:", rel):
        raise ValueError("Bad path.")
    parts = PurePosixPath(rel).parts
    if any(p in ("..", ".") for p in parts):
        raise ValueError("Bad path.")
    root = Path(src).resolve()
    p = (root / Path(*parts)).resolve()
    if root not in p.parents:
        raise ValueError("Bad path.")
    return p


def editable(robot: str, rel: str) -> bool:
    return re.fullmatch(rf"{re.escape(robot)}_tasks/programs/[A-Za-z0-9_]+\.py", rel) is not None


def tree(src, robot: str) -> list[dict]:
    root = Path(src)
    out = []
    for p in root.rglob("*"):
        rel = p.relative_to(root)
        if (p.is_file() and p.suffix in TEXT_EXT and not SKIP_DIRS.intersection(rel.parts)
                and not any(part.startswith(".") for part in rel.parts)):
            r = rel.as_posix()
            out.append({"path": r, "editable": editable(robot, r)})
    return sorted(out, key=lambda f: f["path"])


def read(src, rel: str) -> str:
    p = safe(src, rel)
    if not p.is_file():
        raise FileNotFoundError(rel)
    return p.read_text(encoding="utf-8", errors="replace")


def write(src, robot: str, rel: str, content: str) -> None:
    p = safe(src, rel)
    if not editable(robot, rel):
        raise PermissionError(f"Only {robot}_tasks/programs/*.py can be edited.")
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content, encoding="utf-8", newline="\n")
