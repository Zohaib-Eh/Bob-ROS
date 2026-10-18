import { useEffect, useRef, useState } from "react";
import { browserStore, type Saved, type SaveStore } from "../model/autosave";
import { loadProject, saveProject } from "../model/project";
import type { ModelState } from "../model/types";
import type { Ctx } from "./ctx";

const DELAY_MS = 1500;   // after the last change: a 10 MB save on every click would stutter

/** Keeps the project in browser storage as you work. After a reload with nothing loaded, offers the last save
 *  (Restore / Discard); starting new work instead drops the offer and saving carries on. */
export function useAutosave(model: ModelState, dispatch: Ctx["dispatch"], store: SaveStore = browserStore) {
  const [offer, setOffer] = useState<Saved | null>(null);
  const deciding = useRef(true);   // until the last save has been offered or found absent, don't overwrite it
  const live = useRef(model);
  live.current = model;
  const hasWork = (m: ModelState) => Object.keys(m.bodies).length > 0;

  useEffect(() => {
    void store.read().then((s) => {
      if (s && !hasWork(live.current)) setOffer(s); else deciding.current = false;
    });
  }, [store]);

  useEffect(() => {
    if (deciding.current && offer && hasWork(model)) { setOffer(null); deciding.current = false; }   // new work instead
    if (deciding.current || !hasWork(model)) return;
    const t = setTimeout(() => void store.write({ text: saveProject(model), at: Date.now() }), DELAY_MS);
    return () => clearTimeout(t);
  }, [model, offer, store]);

  return {
    offer,
    restore: () => {
      if (!offer) return;
      try { dispatch({ type: "load", model: loadProject(offer.text) }); } catch { void store.clear(); }
      setOffer(null);
      deciding.current = false;
    },
    discard: () => { void store.clear(); setOffer(null); deciding.current = false; },
  };
}
