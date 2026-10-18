"""The LLM side of the Program step: run_agent() is the only entry point. Its one backend today
is Claude Code in headless mode (`claude -p`) on the user's own subscription; an API backend
goes here when other people use the app."""
import json
import re
import shutil
import subprocess
import threading
from pathlib import Path
from typing import Iterable, Iterator

from app.program.traces import trace_of

SIGN_IN = "Run `claude` once in a terminal to sign in."
NOT_FOUND = "Claude Code was not found. Install it, then run `claude` once in a terminal to sign in."
_SIGNED_OUT = re.compile(r"(invalid api key|not logged in|please run /login)", re.I)


def claude_command(exe: str, robot: str, brief_file: Path, resume: str | None) -> list[str]:
    # dontAsk denies everything not explicitly allowed (bare --allowedTools still lets read-only Bash through)
    cmd = [exe, "-p", "--output-format", "stream-json", "--verbose",
           "--append-system-prompt-file", str(brief_file),
           "--tools", "Read,Write,Edit,Bash",
           "--permission-mode", "dontAsk",
           "--allowedTools", "Read(./**)", f"Read(../{robot}_program/**)",
           "Write(./programs/**)", "Edit(./programs/**)",
           f"Bash(../{robot}_program/run_task.sh:*)"]
    if resume:
        cmd += ["--resume", resume]
    return cmd


def _program_path(file_path: str) -> str | None:
    """'programs/...' tail of a Write/Edit path (Windows or POSIX, absolute or relative), else None."""
    parts = re.split(r"[\\/]+", file_path)
    idx = [i for i, p in enumerate(parts[:-1]) if p == "programs"]
    return "/".join(parts[idx[-1]:]) if idx else None


def _blocks(msg: dict) -> list[dict]:
    content = (msg.get("message") or {}).get("content") or []
    return content if isinstance(content, list) else [{"type": "text", "text": str(content)}]


def _result_text(block: dict) -> str:
    c = block.get("content")
    if isinstance(c, list):
        return "\n".join(b.get("text", "") for b in c if isinstance(b, dict))
    return str(c or "")


def parse_stream(lines: Iterable[str], tasks_dir: Path) -> Iterator[dict]:
    session_id, last_ok, last_program, runs_seen = None, False, None, set()
    for line in lines:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        kind = msg.get("type")
        session_id = msg.get("session_id") or session_id
        if kind == "assistant":
            for b in _blocks(msg):
                if b.get("type") == "text" and b.get("text", "").strip():
                    yield {"kind": "text", "text": b["text"]}
                elif b.get("type") == "tool_use":
                    inp = b.get("input") or {}
                    if b.get("name") in ("Write", "Edit") and inp.get("file_path"):
                        prog = _program_path(inp["file_path"])
                        last_program = prog or last_program
                        yield {"kind": "file", "path": prog or inp["file_path"]}
                    elif b.get("name") == "Bash" and "run_task.sh" in inp.get("command", ""):
                        runs_seen.add(b.get("id"))
                        yield {"kind": "run", "n": len(runs_seen)}
        elif kind == "user":
            for b in _blocks(msg):
                if b.get("type") == "tool_result" and b.get("tool_use_id") in runs_seen:
                    m = re.findall(r"^RESULT: (ok|failed: .*)$", _result_text(b), re.M)
                    if m:
                        last_ok = m[-1] == "ok"
                        yield {"kind": "run_result", "ok": last_ok, "message": m[-1], "output": _result_text(b),
                               "trace": trace_of(_result_text(b))}
        elif kind == "result":
            if msg.get("is_error"):
                text = str(msg.get("result") or msg.get("subtype") or "Claude stopped with an error.")
                yield {"kind": "error", "message": SIGN_IN if _SIGNED_OUT.search(text) else text}
                return
            count = tasks_dir / ".run_count"
            try:
                runs = int(count.read_text().strip() or 0)
            except (OSError, ValueError):
                runs = len(runs_seen)
            code = None
            if last_program and (tasks_dir / last_program).exists():
                code = (tasks_dir / last_program).read_text(encoding="utf-8")
            yield {"kind": "done", "summary": str(msg.get("result") or "").strip(), "runs": runs,
                   "ok": last_ok, "session_id": session_id, "program": last_program, "code": code}
            return


def run_agent(robot: str, src: Path, task: str, resume: str | None = None,
              exe: str | None = None, extra: list[str] | None = None,
              cancel: threading.Event | None = None) -> Iterator[dict]:
    """Stream events for one task. `extra` is for tests (arguments placed before the claude flags).
    Setting `cancel` kills the child, which ends the stream."""
    exe = exe or shutil.which("claude")
    if not exe or not Path(exe).exists():
        yield {"kind": "error", "message": NOT_FOUND}
        return
    tasks_dir = Path(src) / f"{robot}_tasks"
    tasks_dir.mkdir(parents=True, exist_ok=True)
    (tasks_dir / ".run_count").write_text("0")
    brief = Path(src) / f"{robot}_program" / "brief.md"
    cmd = claude_command(exe, robot, brief, resume)
    if extra:
        cmd = [exe, *extra]
    try:
        proc = subprocess.Popen(cmd, cwd=tasks_dir, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True, encoding="utf-8", errors="replace")
    except OSError as exc:
        yield {"kind": "error", "message": f"Could not start Claude Code: {exc}"}
        return
    if cancel is not None:
        def _watch():
            cancel.wait()
            if proc.poll() is None:
                proc.kill()
        threading.Thread(target=_watch, daemon=True).start()
    err_lines: list[str] = []
    drain = threading.Thread(target=lambda: err_lines.extend(proc.stderr), daemon=True)
    drain.start()
    try:
        try:
            proc.stdin.write(task)
            proc.stdin.close()
        except OSError:
            pass  # claude exited early; its stderr explains why
        finished = False
        for ev in parse_stream(proc.stdout, tasks_dir):
            finished = finished or ev["kind"] in ("done", "error")
            yield ev
        code = proc.wait()
        drain.join(timeout=5)
        err = "".join(err_lines)
        if not finished:
            if _SIGNED_OUT.search(err):
                yield {"kind": "error", "message": SIGN_IN}
            else:
                last = err.strip().splitlines()[-1] if err.strip() else f"Claude exited with code {code}."
                yield {"kind": "error", "message": last}
    finally:
        if proc.poll() is None:  # client disconnected mid-run: don't leave claude editing files
            proc.kill()
            proc.wait()
