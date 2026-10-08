import json
import re
from pathlib import Path

from corpus import generate, run

EXPECTED = Path(__file__).resolve().parents[1] / "corpus" / "expected"
LINE = re.compile(r"^\S+ \| parts \d+/≥\d+ \| failed \d+ \| joints \d+/\d+ type \d+/\d+ axis \d+/\d+ "
                  r"\| warn \d+ \| export (ok|FAIL) \| [\d.]+s$")


def test_scorecard_runs_and_detects_regression(tmp_path, capsys):
    corpus = tmp_path / "corpus"; generate.main([str(corpus)])
    baseline, report = tmp_path / "baseline.json", tmp_path / "report.json"
    args = ["--corpus-dir", str(corpus), "--expected-dir", str(EXPECTED),
            "--baseline", str(baseline), "--report", str(report)]

    assert run.main(args + ["--update-baseline"]) == 0
    out = capsys.readouterr().out
    scored = [l for l in out.splitlines() if " | parts " in l]
    assert len(scored) == 3 and all(LINE.match(l) for l in scored)   # irb1300 skipped (no file)
    assert "irb1300: skipped" in out
    data = json.loads(report.read_text())
    assert data["peg_in_hole"]["type_ok"] == 1 and data["peg_in_hole"]["export_ok"] is True

    assert run.main(args) == 0                                        # at baseline

    b = json.loads(baseline.read_text()); b["peg_in_hole"]["type_ok"] = 5
    baseline.write_text(json.dumps(b))
    assert run.main(args) == 1                                        # regressed


def test_ambiguous_and_unknown_body_names_are_reported(tmp_path, capsys):
    corpus, exp = tmp_path / "corpus", tmp_path / "expected"
    generate.main([str(corpus)]); exp.mkdir()
    (exp / "m.yaml").write_text(
        "file: peg_in_hole.step\ngenerated: true\nlinks:\n"
        "  - {name: a, bodies: [nope]}\n")
    args = ["--corpus-dir", str(corpus), "--expected-dir", str(exp),
            "--baseline", str(tmp_path / "b.json"), "--report", str(tmp_path / "r.json")]
    assert run.main(args) == 1
    assert "no body named 'nope'" in capsys.readouterr().out


def test_same_named_bodies_are_ambiguous(tmp_path, monkeypatch, capsys):
    corpus, exp = tmp_path / "corpus", tmp_path / "expected"
    generate.main([str(corpus)]); exp.mkdir()
    real = run.import_path

    def twice(*a, **kw):
        bodies, sc, w = real(*a, **kw)
        return [b.model_copy(update={"name": "dup"}) for b in bodies], sc, w
    monkeypatch.setattr(run, "import_path", twice)
    (exp / "m.yaml").write_text(
        "file: peg_in_hole.step\ngenerated: true\nlinks:\n  - {name: a, bodies: [dup]}\n")
    args = ["--corpus-dir", str(corpus), "--expected-dir", str(exp),
            "--baseline", str(tmp_path / "b.json"), "--report", str(tmp_path / "r.json")]
    assert run.main(args) == 1
    assert "ambiguous body name 'dup' (2 bodies)" in capsys.readouterr().out
