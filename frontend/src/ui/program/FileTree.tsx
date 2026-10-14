import type { FileEntry } from "../../api/program";

const GROUPS: { label: string; match: (p: string) => boolean }[] = [
  { label: "Programs", match: (p) => /_tasks\/programs\//.test(p) },
  { label: "Library", match: (p) => /^[^/]+_program\//.test(p) },
  { label: "Robot", match: () => true },
];

export default function FileTree({ files, open, changed, onOpen }: {
  files: FileEntry[]; open: string | null; changed: Set<string>; onOpen(path: string): void;
}) {
  const seen = new Set<string>();
  return (
    <nav className="file-tree">
      {GROUPS.map((g) => {
        const items = files.filter((f) => !seen.has(f.path) && g.match(f.path));
        items.forEach((f) => seen.add(f.path));
        return items.length ? (
          <div key={g.label}>
            <div className="tree-group">{g.label}</div>
            <ul>
              {items.map((f) => (
                <li key={f.path} title={f.path}
                  className={`${f.path === open ? "open" : ""}${changed.has(f.path) ? " changed" : ""}${f.editable ? "" : " readonly"}`}>
                  <button onClick={() => onOpen(f.path)}>{g.label === "Programs" ? f.path.split("/").pop() : f.path.split("/").slice(1).join("/")}</button>
                </li>
              ))}
            </ul>
          </div>
        ) : null;
      })}
    </nav>
  );
}
