import { useEffect, useState } from "react";
import { suggestEfforts } from "../api/client";
import { buildExportRequest } from "../model/exportRequest";
import { DEFAULT_GRIP_FORCE } from "../model/gripper";
import type { Ctx } from "./ctx";

/** Suggested motor strength per joint id, refreshed 500 ms after the joints or links change. */
export function useEffortSuggestions(ctx: Pick<Ctx, "model" | "robotName">): Record<string, number> {
  const { model, robotName } = ctx;
  const [s, setS] = useState<Record<string, number>>({});
  useEffect(() => {
    if (!model.joints.length) { setS({}); return; }
    const ctl = new AbortController();
    const t = setTimeout(() => {
      const req = buildExportRequest(model);
      // a finger's "Max torque" is its grip force: the default, not an arm-motor estimate
      const fingers = model.gripper ? [model.gripper.leader, model.gripper.follower] : [];
      suggestEfforts(robotName, req.links, req.joints, req.bodiesByLink, ctl.signal)
        .then((r) => setS({ ...r, ...Object.fromEntries(fingers.map((id) => [id, DEFAULT_GRIP_FORCE])) }))
        .catch(() => {});
    }, 500);
    return () => { clearTimeout(t); ctl.abort(); };
  }, [model.joints, model.links, model.gripper, robotName]);  // eslint-disable-line react-hooks/exhaustive-deps
  return s;
}
