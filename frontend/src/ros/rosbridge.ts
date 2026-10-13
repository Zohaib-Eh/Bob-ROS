/** Minimal rosbridge v2 client over the browser WebSocket: subscribe and call services. */
export class RosBridge {
  readonly ready: Promise<void>;
  private ws: WebSocket;
  private nextId = 0;
  private subs = new Map<string, (msg: any) => void>();
  private calls = new Map<string, (m: any) => void>();

  constructor(url: string, Socket: typeof WebSocket = WebSocket) {
    this.ws = new Socket(url);
    this.ready = new Promise((resolve, reject) => {
      this.ws.onopen = () => resolve();
      this.ws.onerror = () => reject(new Error(`Cannot connect to ROS at ${url}`));
    });
    this.ws.onmessage = (e: MessageEvent) => {
      const m = JSON.parse(e.data as string);
      if (m.op === "publish") this.subs.get(m.topic)?.(m.msg);
      else if (m.op === "service_response") { this.calls.get(m.id)?.(m); this.calls.delete(m.id); }
    };
  }

  subscribe(topic: string, type: string, cb: (msg: any) => void, throttleMs = 0) {
    this.subs.set(topic, cb);
    this.send({ op: "subscribe", topic, type, throttle_rate: throttleMs });
  }

  call<T>(service: string, type: string, args: object): Promise<T> {
    const id = `call_${this.nextId++}`;
    return new Promise((resolve, reject) => {
      this.calls.set(id, (m) => (m.result === false ? reject(new Error(`${service} failed`)) : resolve(m.values as T)));
      this.send({ op: "call_service", id, service, type, args });
    });
  }

  close() { this.ws.close(); }

  private send(o: object) { this.ws.send(JSON.stringify(o)); }
}
