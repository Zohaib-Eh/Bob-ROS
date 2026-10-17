import zipfile

from app.ros.cli import main
from tests.ros_helpers import two_joint_zip


def test_cli_writes_the_three_packages(tmp_path, capsys):
    z = two_joint_zip(tmp_path)
    assert main([str(z), "-o", str(tmp_path / "ws"), "--samples", "50"]) == 0
    src = tmp_path / "ws" / "src"
    assert {p.name for p in src.iterdir()} == {"bot", "bot_moveit_config", "bot_kinematics", "bot_program", "bot_tasks",
                                               "cad_to_urdf_glue"}
    assert "bot_moveit_config" in capsys.readouterr().out


def test_cli_rejects_a_zip_that_is_not_an_export(tmp_path, capsys):
    z = tmp_path / "other.zip"
    with zipfile.ZipFile(z, "w") as f:
        f.writestr("readme.txt", "hi")
    assert main([str(z), "-o", str(tmp_path / "ws")]) == 1
    assert "error: Not a cad-to-urdf export." in capsys.readouterr().err
    assert not (tmp_path / "ws").exists()


def test_cli_rejects_a_file_that_is_not_a_zip(tmp_path, capsys):
    f = tmp_path / "robot.zip"
    f.write_text("not a zip")
    assert main([str(f), "-o", str(tmp_path / "ws")]) == 1
    assert "error: Not a cad-to-urdf export." in capsys.readouterr().err


def test_cli_reports_unsupported_robots_in_one_sentence(tmp_path, capsys):
    z = two_joint_zip(tmp_path, name="Bot")
    assert main([str(z), "-o", str(tmp_path / "ws")]) == 1
    err = capsys.readouterr().err
    assert err.startswith("error: Robot name 'Bot' is not a valid ROS package name")
    assert err.count("\n") == 1


def _rezip(src, dest, edit):
    with zipfile.ZipFile(src) as a, zipfile.ZipFile(dest, "w") as b:
        for n in a.namelist():
            b.writestr(n, edit(n, a.read(n)))
    return dest


def test_cli_reports_an_unreadable_mesh(tmp_path, capsys):
    z = _rezip(two_joint_zip(tmp_path), tmp_path / "g.zip",
               lambda n, d: b"garbage" if n.endswith("hand.stl") else d)
    assert main([str(z), "-o", str(tmp_path / "ws")]) == 1
    assert capsys.readouterr().err == "error: Mesh for link 'hand' is empty or unreadable.\n"
    assert not (tmp_path / "ws").exists()


def test_cli_reports_malformed_urdf(tmp_path, capsys):
    z = _rezip(two_joint_zip(tmp_path), tmp_path / "t.zip",
               lambda n, d: d[:100] if n.endswith(".urdf") else d)
    assert main([str(z), "-o", str(tmp_path / "ws")]) == 1
    assert capsys.readouterr().err == "error: The URDF is not valid XML.\n"
    assert not (tmp_path / "ws").exists()
