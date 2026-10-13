import { Play, Save } from "lucide-react";
import { useEffect, useState, type ReactNode } from "react";
import * as api from "../../api/program";
import type { FileEntry } from "../../api/program";
import Editor from "./Editor";
import FileTree from "./FileTree";

interface Props { preview: ReactNode; conversation: string | null; running: boolean; changed: Set<string>; onRan(): void }

export default function Workspace({ preview, conversation, running, changed, onRan }: Props) {
  const [tab, setTab] = useState<"preview" | "code">("preview");
  const [files, setFiles] = useState<FileEntry[]>([]);
  const [file, setFile] = useState<{ path: string; content: string; editable: boolean } | null>(null);
  const [text, setText] = useState("");
  const [note, setNote] = useState<string | null>(null);
  const dirty = !!file && text !== file.content;

  // re-list when a task starts/finishes (it may have written files); listing is read-only so safe while running
  useEffect(() => { if (tab === "code") void api.listFiles().then(setFiles).catch((e) => setNote(e.message)); }, [tab, running]);
  useEffect(() => { if (!running && file && !dirty) void openFile(file.path); }, [running]);  // eslint-disable-line react-hooks/exhaustive-deps

  async function openFile(path: string) {
    try { const f = await api.readFile(path); setFile(f); setText(f.content); setNote(null); }
    catch (e) { setNote((e as Error).message); }
  }
  async function save(): Promise<boolean> {
    if (!file) return false;
    try { await api.saveFile(file.path, text, conversation); setFile({ ...file, content: text }); setNote("Saved"); return true; }
    catch (e) { setNote((e as Error).message); return false; }
  }
  async function run() {
    if (!file) return;
    if (dirty && !(await save())) return;
    try { const r = await api.runFile(file.path, conversation); setNote(r.ok ? "Run ok" : r.output.trim().split("\n").pop() ?? "Run failed"); onRan(); }
    catch (e) { setNote((e as Error).message); }
  }

  return (
    <>
      <div className="tabs" role="tablist">
        <button role="tab" aria-selected={tab === "preview"} className={tab === "preview" ? "active" : ""} onClick={() => setTab("preview")}>Preview</button>
        <button role="tab" aria-selected={tab === "code"} className={tab === "code" ? "active" : ""} onClick={() => setTab("code")}>Code</button>
      </div>
      <div className="tab-body" style={{ display: tab === "preview" ? "flex" : "none" }}>{preview}</div>
      {tab === "code" && (
        <div className="code-tab">
          <FileTree files={files} open={file?.path ?? null} changed={changed} onOpen={(p) => void openFile(p)} />
          <div className="code-pane">
            {file ? (
              <>
                <div className="code-bar">
                  <span className="muted small">{file.path}{dirty ? " •" : ""}{file.editable ? "" : " (generated, read-only)"}</span>
                  <span className="spacer" />
                  {note && <span className="muted small">{note}</span>}
                  {file.editable && <>
                    <button className="btn" onClick={() => void save()} disabled={running || !dirty}><Save size={14} />Save</button>
                    <button className="btn" onClick={() => void run()} disabled={running}><Play size={14} />Run</button>
                  </>}
                </div>
                <Editor value={text} readOnly={!file.editable || running} onChange={setText} />
              </>
            ) : <p className="muted pad">Pick a file. Programs under Programs can be edited and run.</p>}
          </div>
        </div>
      )}
    </>
  );
}
