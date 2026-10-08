import { screen } from "@testing-library/react";
import { useReducer, useState, type ReactNode } from "react";
import { vi } from "vitest";
import type { Body } from "../api/types";
import type { StepId } from "../model/checklist";
import { historyReducer, initialHistory, type History } from "../model/history";
import type { JointDraft } from "../model/jointDraft";
import type { Ctx, PickMode } from "./ctx";
import type { Selection } from "./selection";

export const imported = (bodies: Body[]): History =>
  historyReducer(initialHistory(), { type: "import", bodies, warnings: [] });

/** Real reducer and real UI state around the component under test. */
export function Harness({ start, step = "parts", viewer = null, detecting = false, ui }: {
  start: History; step?: StepId; viewer?: Ctx["viewer"]; detecting?: boolean; ui: (ctx: Ctx) => ReactNode;
}) {
  const [h, dispatch] = useReducer(historyReducer, start);
  const [s, goTo] = useState<StepId>(step);
  const [selection, setSelection] = useState<Selection>(null);
  const [pick, setPick] = useState<PickMode>("select");
  const [jointDraft, setJointDraft] = useState<JointDraft | null>(null);
  const [scale, setScale] = useState(0.001);
  const [robotName, setRobotName] = useState("robot");
  const [autoGrouped, setAutoGrouped] = useState(false);
  const [notify] = useState(() => vi.fn());
  const ctx: Ctx = { model: h.present, dispatch, canUndo: h.past.length > 0, canRedo: h.future.length > 0,
    step: s, goTo, selection, setSelection, pick, setPick, jointDraft, setJointDraft, viewer, notify, scale,
    setScale, robotName, setRobotName, autoGrouped, setAutoGrouped, detecting };
  return (
    <>
      {ui(ctx)}
      <pre data-testid="state">{JSON.stringify({ links: h.present.links, joints: h.present.joints,
        suggestions: h.present.suggestions, ignored: h.present.ignored, rootLinkId: h.present.rootLinkId, step: s,
        selection, pick, jointDraft, scale, robotName, past: h.past.length })}</pre>
    </>
  );
}

export const state = () => JSON.parse(screen.getByTestId("state").textContent!);
