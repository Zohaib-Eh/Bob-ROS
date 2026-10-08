"""Run every model with expectations through the pipeline and score it.
Usage: python -m corpus.run [--corpus-dir DIR] [--expected-dir DIR] [--baseline FILE]
                            [--report FILE] [--update-baseline] [--list-parts NAME]"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import app  # noqa: F401  (BLAS thread cap)
import numpy as np
import yaml

from app.export.urdf_writer import export_package
from app.importers.pipeline import import_path
from app.importers.placement import combine_bodies
from app.joints.face_matching import bodies_in_contact, find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.models import Joint, Link

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
LOWER_IS_WORSE = ("parts", "type_ok", "axis_ok")
_AXIS_TOL_DEG = 5.0


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _axis_ok(got, want) -> bool:
    if got is None:
        return False
    a, b = np.asarray(got, float), np.asarray(want, float)
    cos = abs(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(min(1.0, cos))) <= _AXIS_TOL_DEG


def _confirmed(joint: Joint, exp: dict) -> Joint:
    """Model the user confirming: first expected type, detected axis/origin."""
    t = exp["type"][0]
    extra = {"limit_lower": -math.pi, "limit_upper": math.pi} if t in ("revolute", "prismatic") else {}
    axis = joint.axis or tuple(exp["axis"])
    return joint.model_copy(update={"type": t, "source": "confirmed", "axis": axis, **extra})


def score_model(exp: dict, path: Path) -> dict:
    t0 = time.time()
    bodies, _, file_warnings = import_path(str(path), path.suffix.lower(), exp.get("scale", 0.001))
    s = {"parts": len(bodies), "failed": sum(b.status == "failed" for b in bodies),
         "joints": 0, "type_ok": 0, "axis_ok": 0,
         "warnings": len(file_warnings) + sum(len(b.warnings) for b in bodies),
         "export_ok": False, "seconds": 0.0,
         "statuses": {st: sum(b.status == st for b in bodies) for st in {b.status for b in bodies}}}
    links, errors = {}, []
    for l in exp.get("links", []):
        picked = []
        for n in l["bodies"]:
            m = [b for b in bodies if b.name == n]
            if len(m) != 1:  # names are display-only: never guess between duplicates
                errors.append(f"ambiguous body name '{n}' ({len(m)} bodies)" if m
                              else f"no body named '{n}'")
            else:
                picked.append(m[0])
        if picked:
            links[l["name"]] = combine_bodies(picked, l["name"], l["name"])
    if errors:
        s["error"] = "; ".join(errors)
    joints = []
    for j in exp.get("joints", []):
        a, b = links.get(j["parent"]), links.get(j["child"])
        if a is None or b is None or not bodies_in_contact(a, b):
            continue
        pairs = find_matching_face_pairs(a, b)
        if not pairs:
            continue
        got = classify_joint(pairs, j["parent"], j["child"])
        s["joints"] += 1
        s["type_ok"] += got.type in j["type"]
        s["axis_ok"] += _axis_ok(got.axis, j["axis"])
        joints.append(_confirmed(got, j))
    if links:
        model_links = [Link(id=n, name=n, body_ids=[n], density_kg_m3=2700.0,
                            accept_approximate_mass=True) for n in links]
        try:
            export_package("corpus_robot", model_links, joints, {n: [b] for n, b in links.items()})
            s["export_ok"] = True
        except ValueError as exc:
            s["export_error"] = str(exc)[:200]
    else:
        s["export_ok"] = True  # nothing to export for parts-only expectations
    s["seconds"] = round(time.time() - t0, 1)
    return s


def _line(name: str, exp: dict, s: dict) -> str:
    n = len(exp.get("joints", []))
    return (f"{name} | parts {s['parts']}/≥{exp.get('parts', {}).get('min', 0)} | failed {s['failed']} "
            f"| joints {s['joints']}/{n} type {s['type_ok']}/{n} axis {s['axis_ok']}/{n} "
            f"| warn {s['warnings']} | export {'ok' if s['export_ok'] else 'FAIL'} | {s['seconds']}s")


def _regressions(name: str, s: dict, base: dict | None) -> list[str]:
    if not base:
        return []
    out = [f"{name}: {k} {base[k]} -> {s[k]}" for k in LOWER_IS_WORSE if s[k] < base[k]]
    if s["failed"] > base["failed"]:
        out.append(f"{name}: failed {base['failed']} -> {s['failed']}")
    if base["export_ok"] and not s["export_ok"]:
        out.append(f"{name}: export ok -> FAIL")
    return out


def _expectation_failures(name: str, exp: dict, s: dict) -> list[str]:
    out = []
    if s["parts"] < exp.get("parts", {}).get("min", 0):
        out.append(f"{name}: parts {s['parts']} < min {exp['parts']['min']}")
    for st, mx in exp.get("statuses", {}).items():
        if s.get("statuses", {}).get(st, 0) > mx:
            out.append(f"{name}: {st} {s['statuses'][st]} > max {mx}")
    if s.get("error"):
        out.append(f"{name}: {s['error']}")
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--corpus-dir", default=str(REPO / "corpus"))
    p.add_argument("--expected-dir", default=str(HERE / "expected"))
    p.add_argument("--baseline", default=str(HERE / "baseline.json"))
    p.add_argument("--report", default=str(REPO / "corpus" / "report.json"))
    p.add_argument("--update-baseline", action="store_true")
    p.add_argument("--list-parts")
    a = p.parse_args(argv)
    corpus, baseline_path = Path(a.corpus_dir), Path(a.baseline)

    if a.list_parts:
        exp = yaml.safe_load((Path(a.expected_dir) / f"{a.list_parts}.yaml").read_text())
        path = corpus / exp["file"]
        bodies, _, _ = import_path(str(path), path.suffix.lower(), exp.get("scale", 0.001))
        for b in bodies:
            v = np.asarray(b.vertices) if b.vertices else np.zeros((1, 3))
            print(f"{b.name} | {b.status} | tris {len(b.triangles)} | "
                  f"bbox {v.min(0).round(3).tolist()}..{v.max(0).round(3).tolist()}")
        return 0

    baseline = json.loads(baseline_path.read_text()) if baseline_path.exists() else {}
    scores, problems, refused = {}, [], set()
    for exp_file in sorted(Path(a.expected_dir).glob("*.yaml")):
        name, exp = exp_file.stem, yaml.safe_load(exp_file.read_text())
        path = corpus / exp["file"]
        if not path.exists():
            print(f"{name}: skipped (file {exp['file']} not in {corpus})")
            continue
        if not exp.get("generated") and exp.get("sha256") and _sha256(path) != exp["sha256"]:
            print(f"{name}: skipped (sha256 mismatch)")
            continue
        try:
            s = score_model(exp, path)
        except Exception as exc:  # a crashing model is a result, not a runner crash
            s = {"parts": 0, "failed": 0, "joints": 0, "type_ok": 0, "axis_ok": 0,
                 "warnings": 0, "export_ok": False, "seconds": 0.0,
                 "error": f"{type(exc).__name__}: {exc}"[:200]}
        scores[name] = s
        print(_line(name, exp, s))
        bad = _expectation_failures(name, exp, s)
        if bad:
            refused.add(name)
        problems += _regressions(name, s, baseline.get(name)) + bad

    Path(a.report).parent.mkdir(parents=True, exist_ok=True)
    Path(a.report).write_text(json.dumps(scores, indent=2))
    if a.update_baseline:
        baseline.update({k: {kk: v[kk] for kk in ("parts", "failed", "type_ok", "axis_ok", "export_ok")}
                         for k, v in scores.items() if k not in refused})
        baseline_path.write_text(json.dumps(baseline, indent=2, sort_keys=True))
        print(f"baseline updated: {baseline_path}")
        for line in problems:
            print(("REFUSED " if line.split(":")[0] in refused else "REGRESSION ") + line)
        return 1 if refused else 0
    for line in problems:
        print("REGRESSION " + line)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")  # scorecard has "≥"; Windows default is cp1252
    sys.exit(main())
