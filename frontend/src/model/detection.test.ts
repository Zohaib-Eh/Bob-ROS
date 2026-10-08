import { describe, expect, it, vi } from "vitest";
import type { DetectResponse } from "../api/types";
import { DetectionRunner, pairsToDetect, type PendingPair } from "./detection";
import { modelReducer } from "./reducer";
import { boxBody } from "./testBodies";
import { emptyModel } from "./types";

const NONE: DetectResponse = { joint: null, reason: "no_contact_faces", evidence: null };
const pair = (key: string, m = "m"): PendingPair => ({ key, linkA: "a", linkB: "b", membership: m });
const tick = () => new Promise((r) => setTimeout(r, 0));

describe("pairsToDetect", () => {
  it("queues touching pairs without a current suggestion", () => {
    let m = modelReducer(emptyModel(), { type: "import", warnings: [], bodies: [
      boxBody("a", [0, 0, 0], [1, 1, 1]), boxBody("b", [1, 0, 0], [2, 1, 1]), boxBody("c", [9, 9, 9], [10, 10, 10])] });
    const pending = pairsToDetect(m);
    expect(pending).toHaveLength(1);
    m = modelReducer(m, { type: "setSuggestion", suggestion: { ...pending[0], status: "review", result: NONE } });
    expect(pairsToDetect(m)).toHaveLength(0);
    const target = m.links.find((l) => l.body_ids.includes("a"))!.id;
    const other = m.links.find((l) => l.body_ids.includes("c"))!.id;
    m = modelReducer(m, { type: "merge", linkIds: [target, other] });   // membership changed
    expect(pairsToDetect(m)).toHaveLength(1);
  });
});

describe("DetectionRunner", () => {
  it("runs pairs one at a time and reports progress", async () => {
    let inFlight = 0, maxInFlight = 0;
    const detect = vi.fn(async () => { inFlight++; maxInFlight = Math.max(maxInFlight, inFlight);
      await tick(); inFlight--; return NONE; });
    const results: string[] = [];
    const progress = vi.fn();
    const r = new DetectionRunner(detect, { onResult: (p) => results.push(p.key), onProgress: progress, onError: vi.fn() });
    r.update([pair("x"), pair("y")], () => []);
    await vi.waitFor(() => expect(results).toEqual(["x", "y"]));
    expect(maxInFlight).toBe(1);
    expect(progress).toHaveBeenLastCalledWith({ done: 2, total: 2, running: false, paused: false });
  });

  it("aborts the running pair when its membership changes and drops its result", async () => {
    const signals: AbortSignal[] = [];
    const detect = vi.fn((_a, _b, _la, _lb, signal: AbortSignal) => {
      signals.push(signal);
      return new Promise<DetectResponse>((resolve, reject) => {
        signal.addEventListener("abort", () => reject(new DOMException("aborted", "AbortError")));
        setTimeout(() => resolve(NONE), 20);
      });
    });
    const results: PendingPair[] = [];
    const onError = vi.fn();
    const r = new DetectionRunner(detect, { onResult: (p) => results.push(p), onProgress: vi.fn(), onError });
    r.update([pair("x", "m1")], () => []);
    await tick();
    r.update([pair("x", "m2")], () => []);
    await vi.waitFor(() => expect(results).toHaveLength(1));
    expect(signals[0].aborted).toBe(true);
    expect(results[0].membership).toBe("m2");
    expect(onError).not.toHaveBeenCalled();
  });

  it("cancel pauses; resume continues; errors pause with a message", async () => {
    const detect = vi.fn().mockRejectedValueOnce(new Error("backend down")).mockResolvedValue(NONE);
    const results: string[] = [];
    const onError = vi.fn();
    const r = new DetectionRunner(detect, { onResult: (p) => results.push(p.key), onProgress: vi.fn(), onError });
    r.update([pair("x")], () => []);
    await vi.waitFor(() => expect(onError).toHaveBeenCalledWith("backend down"));
    expect(results).toEqual([]);
    r.resume();
    await vi.waitFor(() => expect(results).toEqual(["x"]));
  });
});
