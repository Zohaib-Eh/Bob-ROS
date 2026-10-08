import { Circle, Lock, MoveHorizontal, RotateCw, Square } from "lucide-react";
import type { JointType } from "../api/types";

export function JointIcon({ type, size = 13 }: { type: JointType; size?: number }) {
  if (type === "fixed") return <Lock size={size} aria-label="rigid" />;
  if (type === "revolute" || type === "continuous") return <RotateCw size={size} aria-label="revolute" />;
  if (type === "prismatic") return <MoveHorizontal size={size} aria-label="slider" />;
  if (type === "planar") return <Square size={size} aria-label="planar" />;
  return <Circle size={size} aria-label={type} />;
}
