import { useEffect, useState } from "react";

/** Whole seconds since `active` turned true; 0 while inactive. Ticks once a second. */
export function useElapsed(active: boolean): number {
  const [s, setS] = useState(0);
  useEffect(() => {
    setS(0);
    if (!active) return;
    const t0 = Date.now();
    const t = setInterval(() => setS(Math.floor((Date.now() - t0) / 1000)), 1000);
    return () => clearInterval(t);
  }, [active]);
  return s;
}

export const fmtElapsed = (s: number) => `${Math.floor(s / 60)}:${String(s % 60).padStart(2, "0")}`;
