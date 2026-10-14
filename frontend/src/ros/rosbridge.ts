/** Minimal rosbridge v2 client over the browser WebSocket: subscribe and call services. */
export class RosBridge {
  readonly ready: Promise<void>;
  private ws: WebSocket;
  private nextId = 0;
  private subs = new Map<string, (msg: any) => void>();
  private calls = new Map<string, { resolve: (m: any) => void; reject: (e: Error) => void }>();
  private open = false;
  private closed = false;

  constructor(url: string, Socket: typeof WebSocket = WebSocket) {
    this.ws = new Socket(url);
    this.ready = new Promise((resolve, reject) => {
      this.ws.onopen = () => { this.open = true; resolve(); };
      this.ws.onerror = () => { reject(new Error(`Cannot connect to ROS at ${url}`)); this.failCalls(); };
    });
    this.ready.catch(() => {}); // callers that never await ready must not trigger unhandled rejections
    this.ws.onclose = () => this.failCalls();
    this.ws.onmessage = (e: MessageEvent) => {
      let m: any;
      try { m = JSON.parse(e.data as string); } catch { return; }
      if (!m || typeof m !== "object") return;
      if (m.op === "publish") this.subs.get(m.topic)?.(m.msg);
      else if (m.op === "service_response") { this.calls.get(m.id)?.resolve(m); this.calls.delete(m.id); }
    };
  }

  subscribe(topic: string, type: string, cb: (msg: any) => void, throttleMs = 0) {
    this.subs.set(topic, cb);
    this.whenOpen(() => this.send({ op: "subscribe", topic, type, throttle_rate: throttleMs }));
  }

  call<T>(service: string, type: string, args: object): Promise<T> {
    const id = `call_${this.nextId++}`;
    return new Promise((resolve, reject) => {
      this.calls.set(id, {
        resolve: (m) => (m.result === false ? reject(new Error(`${service} failed`)) : resolve(m.values as T)),
        reject,
      });
      this.whenOpen(() => this.send({ op: "call_service", id, service, type, args }), (e) => { this.calls.delete(id); reject(e); });
    });
  }

  close() { this.ws.close(); this.failCalls(); }

  private failCalls() {
    this.closed = true; this.open = false;
    for (const c of this.calls.values()) c.reject(new Error("ROS connection closed"));
    this.calls.clear();
  }

  /** Run now if the socket is open, else once it opens; `onFail` if it never does or send throws. */
  private whenOpen(fn: () => void, onFail: (e: Error) => void = () => {}) {
    const run = () => { try { fn(); } catch (e) { onFail(e as Error); } };
    if (this.closed) onFail(new Error("ROS connection closed"));
    else if (this.open) run(); else this.ready.then(run, onFail);
  }

  private send(o: object) { this.ws.send(JSON.stringify(o)); }
}
