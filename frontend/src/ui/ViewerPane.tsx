import { useEffect, useRef } from "react";
import type { Matrix4 } from "three";
import type { Body, Joint } from "../api/types";
import { Viewer } from "../viewer/Viewer";

interface Props {
  bodies: Body[];
  ignored: string[];
  selectedBodyIds: string[];
  joints: { joint: Joint; dashed: boolean }[];
  preview: { bodyIds: string[]; matrix: Matrix4 } | null;
  onPick: (hit: { bodyId: string; triangle: number } | null, additive: boolean) => void;
  onViewer?: (v: Viewer | null) => void;
}

export default function ViewerPane(p: Props) {
  const ref = useRef<HTMLDivElement>(null);
  const viewer = useRef<Viewer | null>(null);
  useEffect(() => {
    viewer.current = new Viewer(ref.current!);
    p.onViewer?.(viewer.current);
    return () => { viewer.current?.dispose(); viewer.current = null; p.onViewer?.(null); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);
  useEffect(() => { viewer.current?.onPick(p.onPick); }, [p.onPick]);
  useEffect(() => { viewer.current?.setBodies(p.bodies, new Set(p.ignored)); }, [p.bodies, p.ignored]);
  useEffect(() => { viewer.current?.setSelection(p.selectedBodyIds); }, [p.selectedBodyIds]);
  useEffect(() => { viewer.current?.setJoints(p.joints); }, [p.joints]);
  useEffect(() => { viewer.current?.setPreview(p.preview?.bodyIds ?? [], p.preview?.matrix ?? null); }, [p.preview]);
  return <div className="viewer" ref={ref} data-testid="viewer" />;
}
