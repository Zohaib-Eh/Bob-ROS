import type { Body, DetectResponse } from "../api/types";
import { membership, pairKey, touchingPairs } from "./pairs";
import type { ModelState } from "./types";

export interface PendingPair { key: string; linkA: string; linkB: string; membership: string }
export type DetectFn = (a: Body[], b: Body[], linkA: string, linkB: string, signal: AbortSignal) => Promise<DetectResponse>;
export interface Progress { done: number; total: number; running: boolean; paused: boolean }

export function pairsToDetect(m: ModelState): PendingPair[] {
  return touchingPairs(m.links, m.bodies)
    .map(([a, b]) => ({ key: pairKey(a.id, b.id), linkA: a.id, linkB: b.id, membership: membership(a, b) }))
    .filter((p) => m.suggestions[p.key]?.membership !== p.membership);
}

interface Handlers {
  onResult(p: PendingPair, r: DetectResponse): void;
  onProgress(p: Progress): void;
  onError(message: string): void;
}

const same = (a: PendingPair, b: PendingPair) => a.key === b.key && a.membership === b.membership;

export class DetectionRunner {
  private pending: PendingPair[] = [];
  private current: { pair: PendingPair; ctrl: AbortController } | null = null;
  private bodiesOf: (linkId: string) => Body[] = () => [];
  private paused = false;
  private done = 0;

  constructor(private detect: DetectFn, private handlers: Handlers) {}

  update(pairs: PendingPair[], bodiesOf: (linkId: string) => Body[]) {
    this.bodiesOf = bodiesOf;
    this.pending = pairs.filter((p) => !(this.current && same(p, this.current.pair)));
    if (this.current && !pairs.some((p) => same(p, this.current!.pair))) {
      this.current.ctrl.abort();
      this.current = null;
    }
    if (!this.pending.length && !this.current) this.done = 0;
    this.report();
    void this.kick();
  }

  cancel() {
    this.paused = true;
    if (this.current) {
      this.pending.unshift(this.current.pair);
      this.current.ctrl.abort();
      this.current = null;
    }
    this.report();
  }

  resume() {
    this.paused = false;
    this.report();
    void this.kick();
  }

  private report() {
    const running = this.current !== null;
    this.handlers.onProgress({
      done: this.done, total: this.done + this.pending.length + (running ? 1 : 0), running, paused: this.paused });
  }

  private async kick() {
    if (this.paused || this.current || !this.pending.length) return;
    const pair = this.pending.shift()!;
    const ctrl = new AbortController();
    this.current = { pair, ctrl };
    this.report();
    try {
      const result = await this.detect(this.bodiesOf(pair.linkA), this.bodiesOf(pair.linkB), pair.linkA, pair.linkB, ctrl.signal);
      if (this.current?.ctrl !== ctrl) return;   // superseded while running
      this.current = null;
      this.done++;
      this.handlers.onResult(pair, result);
    } catch (e) {
      if (ctrl.signal.aborted) return;
      if (this.current?.ctrl === ctrl) this.current = null;
      this.pending.unshift(pair);
      this.paused = true;
      this.handlers.onError((e as Error).message);
    }
    this.report();
    void this.kick();
  }
}
