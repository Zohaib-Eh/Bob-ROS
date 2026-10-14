import { Plus, Trash2 } from "lucide-react";
import { useState } from "react";
import type { ConversationInfo } from "../../api/program";

interface Props {
  items: ConversationInfo[]; active: string | null; runningId: string | null;
  onSelect(id: string): void; onNew(): void; onRename(id: string, title: string): void; onDelete(id: string): void;
}

export default function ConversationList({ items, active, runningId, onSelect, onNew, onRename, onDelete }: Props) {
  const [editing, setEditing] = useState<string | null>(null);
  const [draft, setDraft] = useState("");
  return (
    <nav className="conversations">
      <button className="btn" onClick={onNew}><Plus size={14} />New</button>
      <ul>
        {items.map((c) => (
          <li key={c.id} data-testid={`conv-${c.id}`} className={`${c.id === active ? "active" : ""}${c.broken ? " broken" : ""}`}>
            {editing === c.id ? (
              <input autoFocus value={draft} onChange={(e) => setDraft(e.target.value)}
                onBlur={() => setEditing(null)}
                onKeyDown={(e) => {
                  if (e.key === "Enter" && draft.trim()) { onRename(c.id, draft.trim()); setEditing(null); }
                  if (e.key === "Escape") setEditing(null);
                }} />
            ) : (
              <button className="conv-title" disabled={c.broken} onClick={() => onSelect(c.id)}
                onDoubleClick={() => { if (!c.broken) { setEditing(c.id); setDraft(c.title); } }}>{c.title}</button>
            )}
            {c.id === runningId
              ? <span className="running-dot" aria-label="Task running" />
              : <button className="icon" aria-label={`Delete ${c.title}`}
                  onClick={() => { if (window.confirm(`Delete "${c.title}"?`)) onDelete(c.id); }}><Trash2 size={12} /></button>}
          </li>
        ))}
      </ul>
    </nav>
  );
}
