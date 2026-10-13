"""Conversations of the Program workspace: one JSON file per conversation in the robot's workspace
(<workspace>/conversations/<id>.json). The message format is mirrored by frontend/src/model/chat.ts."""
import json
import re
import time
import uuid
from pathlib import Path

ID_RE = re.compile(r"^[0-9a-f]{12}$")


def folder(src) -> Path:
    return Path(src).parent / "conversations"


def _path(src, cid: str) -> Path:
    if not ID_RE.match(cid):
        raise ValueError("Bad conversation id.")
    return folder(src) / f"{cid}.json"


def title_from(task: str) -> str:
    return " ".join(task.split()[:6])[:60] or "New conversation"


def list_all(src) -> list[dict]:
    out = []
    d = folder(src)
    for p in sorted(d.glob("*.json")) if d.exists() else []:
        if not ID_RE.match(p.stem):
            continue
        try:
            c = json.loads(p.read_text(encoding="utf-8"))
            out.append({k: c[k] for k in ("id", "title", "created", "updated")})
        except (ValueError, KeyError, TypeError):
            out.append({"id": p.stem, "title": "Unreadable conversation", "broken": True})
    return sorted(out, key=lambda c: c.get("updated", 0), reverse=True)


def save(src, conv: dict) -> None:
    conv["updated"] = time.time()
    p = _path(src, conv["id"])
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(conv, indent=1), encoding="utf-8")
    tmp.replace(p)


def create(src, title: str = "New conversation") -> dict:
    now = time.time()
    conv = {"id": uuid.uuid4().hex[:12], "title": title, "created": now, "updated": now,
            "claude_session_id": None, "edited": [], "messages": []}
    save(src, conv)
    return conv


def load(src, cid: str) -> dict:
    p = _path(src, cid)
    if not p.exists():
        raise KeyError(cid)
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except ValueError:
        raise KeyError(cid)


def rename(src, cid: str, title: str) -> dict:
    conv = load(src, cid)
    conv["title"] = title.strip()[:80] or conv["title"]
    save(src, conv)
    return conv


def delete(src, cid: str) -> None:
    p = _path(src, cid)
    if not p.exists():
        raise KeyError(cid)
    p.unlink()


def fold(events: list[dict]) -> dict:
    """Agent events -> one Claude message. The final answer appears once: a trailing narration
    text equal to the summary is dropped."""
    m = {"role": "claude", "items": [], "summary": "", "result": None, "error": None, "program": None}
    for e in events:
        k = e.get("kind")
        if k == "text":
            m["items"].append({"kind": "text", "text": e["text"]})
        elif k == "file":
            if not (m["items"] and m["items"][-1] == {"kind": "file", "path": e["path"]}):
                m["items"].append({"kind": "file", "path": e["path"]})
        elif k == "run":
            m["items"].append({"kind": "run", "n": e["n"]})
        elif k == "run_result":
            for it in reversed(m["items"]):
                if it["kind"] == "run":
                    it.update(ok=e["ok"], message=e["message"], output=e.get("output", ""))
                    break
        elif k == "done":
            m["summary"] = e.get("summary", "")
            m["result"] = {"ok": e.get("ok", False), "runs": e.get("runs", 0)}
            m["program"] = e.get("program")
            while (m["items"] and m["items"][-1]["kind"] == "text"
                   and m["items"][-1]["text"].strip() == m["summary"].strip()):
                m["items"].pop()
        elif k == "error":
            m["error"] = e.get("message", "")
    return m
