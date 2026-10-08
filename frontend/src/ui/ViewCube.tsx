import type { ViewName } from "../viewer/viewMath";
import type { Viewer } from "../viewer/Viewer";

const VIEWS: [ViewName | "home", string][] = [
  ["top", "Top"], ["front", "Front"], ["right", "Right"], ["left", "Left"], ["back", "Back"], ["iso", "Iso"], ["home", "Home"],
];

export default function ViewCube({ viewer }: { viewer: Viewer | null }) {
  return (
    <div className="viewcube" role="toolbar" aria-label="Views">
      {VIEWS.map(([v, label]) => (
        <button key={v} title={v === "home" ? "Fit everything" : `${label} view`} onClick={() => viewer?.setView(v)}>{label}</button>
      ))}
    </div>
  );
}
