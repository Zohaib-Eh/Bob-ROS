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


def test_export_dir_writes_one_zip_per_exported_model(tmp_path):
    corpus = tmp_path / "corpus"; generate.main([str(corpus)])
    out = tmp_path / "zips"
    run.main(["--corpus-dir", str(corpus), "--expected-dir", str(EXPECTED),
              "--baseline", str(tmp_path / "b.json"), "--report", str(tmp_path / "r.json"),
              "--export-dir", str(out)])
    import zipfile
    names = zipfile.ZipFile(out / "peg_in_hole.zip").namelist()
    assert "peg_in_hole/urdf/peg_in_hole.urdf" in names


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


def _one_model(tmp_path, yaml_text, name="open_shell.brep"):
    corpus, exp = tmp_path / "corpus", tmp_path / "expected"
    generate.main([str(corpus)]); exp.mkdir()
    (exp / "m.yaml").write_text(yaml_text.replace("FILE", name))
    return ["--corpus-dir", str(corpus), "--expected-dir", str(exp),
            "--baseline", str(tmp_path / "b.json"), "--report", str(tmp_path / "r.json")]


def test_status_upper_bound_overrun_fails(tmp_path, capsys):
    args = _one_model(tmp_path, "file: FILE\ngenerated: true\nscale: 1.0\nstatuses: {approximate: 0}\n")
    assert run.main(args) == 1
    assert "m: approximate 1 > max 0" in capsys.readouterr().out


def test_update_baseline_refuses_bad_models(tmp_path, capsys):
    args = _one_model(tmp_path, "file: FILE\ngenerated: true\nscale: 1.0\nparts: {min: 5}\n")
    assert run.main(args + ["--update-baseline"]) == 1
    assert "parts 2 < min 5" in capsys.readouterr().out
    assert "m" not in json.loads((tmp_path / "b.json").read_text())


def test_update_baseline_still_prints_regressions(tmp_path, capsys):
    corpus, exp = tmp_path / "corpus", tmp_path / "expected"
    generate.main([str(corpus)]); exp.mkdir()
    (exp / "m.yaml").write_text("file: box_coarse.stl\ngenerated: true\nparts: {min: 1}\n")
    baseline = tmp_path / "b.json"
    args = ["--corpus-dir", str(corpus), "--expected-dir", str(exp),
            "--baseline", str(baseline), "--report", str(tmp_path / "r.json")]
    assert run.main(args + ["--update-baseline"]) == 0
    b = json.loads(baseline.read_text()); b["m"]["parts"] = 5
    baseline.write_text(json.dumps(b))
    capsys.readouterr()
    assert run.main(args + ["--update-baseline"]) == 0
    assert "REGRESSION m: parts 5 -> 1" in capsys.readouterr().out

