"""Run replay check: the run's output names a trace that exists, holds frames and a scene, and ends with the
run's own result. Usage: python3 trace_check.py <tasks dir> <run output file> [physics]"""
import json
import re
import sys
from pathlib import Path

tasks, out = Path(sys.argv[1]), Path(sys.argv[2]).read_text()
physics = len(sys.argv) > 3
found = re.findall(r"^TRACE: (runs/\S+\.json)$", out, re.M)
result = re.findall(r"^RESULT: (.*)$", out, re.M)
if not found:
    sys.exit(f"no trace: {(re.findall(r'^TRACE: .*$', out, re.M) or ['no TRACE line'])[-1]}")
d = json.loads((tasks / found[-1]).read_text())
problems = []
if len(d["frames"]) < 10:
    problems.append(f"only {len(d['frames'])} frames")
if not d["scenes"]:
    problems.append("no scene")
if result and d["result"] != result[-1]:
    problems.append(f"result {d['result']!r} vs RESULT {result[-1]!r}")
meta = Path("/tmp/sim/meta.json")
loose = physics and meta.exists() and any(o["name"] not in json.loads(meta.read_text())["fixed"]
                                          for o in json.loads(meta.read_text())["objects"])
if loose and not any(f["poses"] for f in d["frames"]):   # only loose objects have MuJoCo poses
    problems.append("no MuJoCo object poses")
if problems:
    sys.exit("; ".join(problems))
kb = (tasks / found[-1]).stat().st_size // 1024
print(f"{found[-1]}: {len(d['frames'])} frames over {d['frames'][-1]['t']:.1f} s, {len(d['scenes'])} scenes, {kb} KB, result {d['result']}")
