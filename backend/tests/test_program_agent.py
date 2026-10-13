import sys
import threading
from pathlib import Path

from app.program import agent

FIXTURE = Path(__file__).parent / "fixtures" / "claude_stream.jsonl"


def test_command_restricts_claude_to_programs_and_run_task(tmp_path):
    cmd = agent.claude_command("claude", "bot", tmp_path / "brief.md", None)
    assert cmd[:2] == ["claude", "-p"] and "stream-json" in cmd
    assert "Bash(../bot_program/run_task.sh:*)" in cmd and "Write(./programs/**)" in cmd
    assert cmd[cmd.index("--permission-mode") + 1] == "dontAsk"
    assert "Read" not in cmd and "Read(./**)" in cmd and "Read(../bot_program/**)" in cmd
    assert "--resume" not in cmd
    assert agent.claude_command("claude", "bot", tmp_path / "b.md", "abc")[-2:] == ["--resume", "abc"]


def test_program_path_takes_the_last_programs_segment():
    f = agent._program_path
    assert f(r"C:\Users\me\x_tasks\programs\task_001.py") == "programs/task_001.py"
    assert f("programs/x.py") == "programs/x.py"
    assert f("/home/me/x_tasks/programs/task_002.py") == "programs/task_002.py"
    assert f("/tmp/other.py") is None


def test_parse_stream_turns_the_recorded_session_into_events(tmp_path):
    (tmp_path / "programs").mkdir()
    (tmp_path / "programs" / "task_001.py").write_text('print("hello")\n', encoding="utf-8")
    (tmp_path / ".run_count").write_text("1")
    events = list(agent.parse_stream(FIXTURE.read_text(encoding="utf-8").splitlines(), tmp_path))
    kinds = [e["kind"] for e in events]
    assert "text" in kinds and "file" in kinds and "run_result" in kinds
    assert kinds.count("run") == 1  # `ls /` is not a run
    done = events[-1]
    assert done["kind"] == "done" and done["session_id"] and done["runs"] == 1 and done["ok"] is True
    assert done["summary"]
    assert done["program"] == "programs/task_001.py" and done["code"] == 'print("hello")\n'


def test_missing_claude_is_one_error_event(tmp_path):
    events = list(agent.run_agent("bot", tmp_path, "do it", exe=str(tmp_path / "nope.exe")))
    assert events == [{"kind": "error", "message": "Claude Code was not found. Install it, then run `claude` once in a terminal to sign in."}]


def test_signed_out_claude_maps_to_sign_in_message(tmp_path):
    (tmp_path / "bot_tasks").mkdir()
    fake = tmp_path / "fake_claude.py"
    fake.write_text("import sys; sys.stdin.read(); print('Invalid API key · Please run /login', file=sys.stderr); sys.exit(1)", encoding="utf-8")
    events = list(agent.run_agent("bot", tmp_path, "do it", exe=sys.executable, extra=[str(fake)]))
    assert events[-1] == {"kind": "error", "message": "Run `claude` once in a terminal to sign in."}


def _fake(tmp_path, body):
    (tmp_path / "bot_tasks").mkdir(exist_ok=True)
    f = tmp_path / "fake.py"
    f.write_text(body, encoding="utf-8")
    return [str(f)]


def test_closing_the_generator_kills_the_child(tmp_path, monkeypatch):
    import subprocess
    procs = []
    real = subprocess.Popen
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: procs.append(real(*a, **k)) or procs[-1])
    extra = _fake(tmp_path, "import time\nprint('{\"type\":\"assistant\",\"message\":{\"content\":[{\"type\":\"text\",\"text\":\"hi\"}]}}', flush=True)\ntime.sleep(30)")
    gen = agent.run_agent("bot", tmp_path, "x", exe=sys.executable, extra=extra)
    assert next(gen)["kind"] == "text"
    gen.close()
    assert procs[0].poll() is not None


def test_child_exiting_without_reading_stdin_is_one_error_event(tmp_path):
    extra = _fake(tmp_path, "import sys; print('boom', file=sys.stderr); sys.exit(1)")
    events = list(agent.run_agent("bot", tmp_path, "x" * 1_000_000, exe=sys.executable, extra=extra))
    assert [e["kind"] for e in events] == ["error"] and events[0]["message"] == "boom"


def test_usage_limit_error_passes_through():
    line = '{"type":"result","is_error":true,"result":"Claude AI usage limit reached|1791900000"}'
    events = list(agent.parse_stream([line], Path(".")))
    assert events == [{"kind": "error", "message": "Claude AI usage limit reached|1791900000"}]


def test_cancel_kills_the_child_and_ends_the_stream(tmp_path, monkeypatch):
    import subprocess
    procs = []
    real = subprocess.Popen
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **k: procs.append(real(*a, **k)) or procs[-1])
    extra = _fake(tmp_path, "import time\nprint('{\"type\":\"assistant\",\"message\":{\"content\":[{\"type\":\"text\",\"text\":\"hi\"}]}}', flush=True)\ntime.sleep(30)")
    cancel = threading.Event()
    gen = agent.run_agent("bot", tmp_path, "x", exe=sys.executable, extra=extra, cancel=cancel)
    assert next(gen)["kind"] == "text"
    cancel.set()
    drained = []
    t = threading.Thread(target=lambda: drained.extend(gen), daemon=True)
    t.start()
    t.join(10)
    assert not t.is_alive(), "stream did not end after cancel"
    assert procs[0].poll() is not None
