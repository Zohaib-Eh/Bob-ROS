import type { Progress } from "../model/detection";

export default function StatusBar({ progress, onCancel, onResume }:
  { progress: Progress; onCancel: () => void; onResume: () => void }) {
  const { done, total, running, paused } = progress;
  return (
    <div className="statusbar">
      {total > 0 && <span>Checking joints: {done} of {total}</span>}
      {running && !paused && <button onClick={onCancel}>Cancel</button>}
      {paused && total > done && <button onClick={onResume}>Resume</button>}
    </div>
  );
}
