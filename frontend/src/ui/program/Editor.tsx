import { python } from "@codemirror/lang-python";
import { EditorView, basicSetup } from "codemirror";
import { EditorState } from "@codemirror/state";
import { useEffect, useRef } from "react";

/** CodeMirror 6 with Python highlighting. `value` replaces the document when it changes from outside. */
export default function Editor({ value, readOnly, onChange }: { value: string; readOnly: boolean; onChange(v: string): void }) {
  const host = useRef<HTMLDivElement>(null);
  const view = useRef<EditorView | null>(null);
  const change = useRef(onChange);
  change.current = onChange;
  useEffect(() => {
    view.current = new EditorView({
      parent: host.current!,
      state: EditorState.create({ doc: value, extensions: [basicSetup, python(), EditorState.readOnly.of(readOnly),
        EditorView.updateListener.of((u) => { if (u.docChanged) change.current(u.state.doc.toString()); })] }),
    });
    return () => view.current?.destroy();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [readOnly]);
  useEffect(() => {
    const v = view.current;
    if (v && v.state.doc.toString() !== value) v.dispatch({ changes: { from: 0, to: v.state.doc.length, insert: value } });
  }, [value]);
  return <div className="editor" ref={host} />;
}
