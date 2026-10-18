import type { Saved } from "../model/autosave";

/** "Restore your unsaved work from 18:52?" after a page reload (see useAutosave). */
export default function RestoreBanner({ offer, onRestore, onDiscard }: {
  offer: Saved | null; onRestore: () => void; onDiscard: () => void;
}) {
  if (!offer) return null;
  const when = new Date(offer.at).toLocaleString([], { hour: "2-digit", minute: "2-digit", day: "numeric", month: "short" });
  return (
    <div className="banner" role="status">
      <span>Restore your unsaved work from {when}?</span>
      <span className="spacer" />
      <button className="btn btn-primary" onClick={onRestore}>Restore</button>
      <button className="btn" onClick={onDiscard}>Discard</button>
    </div>
  );
}
