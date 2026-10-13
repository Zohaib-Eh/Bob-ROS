import { describe, expect, it } from "vitest";
import { RosBridge } from "./rosbridge";

class FakeSocket {
  static last: FakeSocket;
  sent: any[] = [];
  onopen?: () => void; onerror?: () => void; onclose?: () => void; onmessage?: (e: { data: string }) => void;
  constructor(public url: string) { FakeSocket.last = this; setTimeout(() => this.onopen?.(), 0); }
  send(s: string) { this.sent.push(JSON.parse(s)); }
  close() {}
  emit(o: object) { this.onmessage?.({ data: JSON.stringify(o) }); }
}

describe("RosBridge", () => {
  it("subscribes, delivers messages and answers service calls", async () => {
    const ros = new RosBridge("ws://x", FakeSocket as unknown as typeof WebSocket);
    await ros.ready;
    const got: any[] = [];
    ros.subscribe("/tf", "tf2_msgs/msg/TFMessage", (m) => got.push(m), 50);
    expect(FakeSocket.last.sent[0]).toEqual({ op: "subscribe", topic: "/tf", type: "tf2_msgs/msg/TFMessage", throttle_rate: 50 });
    FakeSocket.last.emit({ op: "publish", topic: "/tf", msg: { a: 1 } });
    expect(got).toEqual([{ a: 1 }]);
    const p = ros.call<{ scene: number }>("/get_planning_scene", "moveit_msgs/srv/GetPlanningScene", { components: { components: 20 } });
    const req = FakeSocket.last.sent[1];
    expect(req.op).toBe("call_service");
    FakeSocket.last.emit({ op: "service_response", id: req.id, result: true, values: { scene: 7 } });
    expect(await p).toEqual({ scene: 7 });
  });

  it("rejects pending calls when the socket closes, and ignores malformed frames", async () => {
    const ros = new RosBridge("ws://x", FakeSocket as unknown as typeof WebSocket);
    await ros.ready;
    const p = ros.call("/s", "t/srv/T", {});
    expect(() => FakeSocket.last.onmessage?.({ data: "not json" })).not.toThrow();
    FakeSocket.last.onclose?.();
    await expect(p).rejects.toThrow("ROS connection closed");
  });

  it("sends a subscribe made before the socket opens once it opens", async () => {
    const ros = new RosBridge("ws://x", FakeSocket as unknown as typeof WebSocket);
    ros.subscribe("/tf", "tf2_msgs/msg/TFMessage", () => {});
    expect(FakeSocket.last.sent).toEqual([]);
    await ros.ready;
    await Promise.resolve();
    expect(FakeSocket.last.sent).toHaveLength(1);
  });
});
