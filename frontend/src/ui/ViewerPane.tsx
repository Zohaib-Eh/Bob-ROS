import { useEffect, useRef } from "react";
import type { Body, Vec3 } from "../api/types";
import type { JointGlyph } from "../viewer/overlays";
import { Viewer, type Hit } from "../viewer/Viewer";

interface Props {
  bodies: Body[];
  ignored: string[];
  selected: string[];
  tints: Record<string, number>;
  approximate: string[];
  glyphs: JointGlyph[];
  loose: [Vec3, Vec3][];
  onPick: (hit: Hit, additive: boolean) => void;
  onViewer: (v: Viewer | null) => void;
}

export default function ViewerPane(p: Props) {
  const ref = useRef<HTMLDivElement>(null);
  const viewer = useRef<Viewer | null>(null);
  useEffect(() => {
    const v = new Viewer(ref.current!);
    viewer.current = v;
    p.onViewer(v);
    return () => { v.dispose(); viewer.current = null; p.onViewer(null); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  useEffect(() => { viewer.current?.onPick(p.onPick); }, [p.onPick]);
  useEffect(() => { viewer.current?.setBodies(p.bodies, new Set(p.ignored)); }, [p.bodies, p.ignored]);
  useEffect(() => { viewer.current?.setSelection(p.selected); }, [p.selected]);
  useEffect(() => { viewer.current?.setTints(p.tints); }, [p.tints]);
  useEffect(() => { viewer.current?.setApproximate(p.approximate); }, [p.approximate]);
  useEffect(() => { viewer.current?.setJointGlyphs(p.glyphs); }, [p.glyphs]);
  useEffect(() => { viewer.current?.setLooseLines(p.loose); }, [p.loose]);
  return <div className="viewer" ref={ref} data-testid="viewer" />;
}
