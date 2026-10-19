import { act, fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it, vi } from "vitest";
import { applyEvent, emptyClaude, type Message } from "../../model/chat";
import type { Trace } from "../../model/replay";
import type { Viewer } from "../../viewer/Viewer";
import Chat from "./Chat";
import Replay from "./Replay";

const trace: Trace = {
  version: 1, robot: "bot", program: "programs/task_001.py", base: "base", result: "failed: Unreachable: too far",
  static: [], frames: [{ t: 0, tf: [], poses: {} }, { t: 2, tf: [], poses: {} }], scenes: [],
};

describe("run replay in the chat", () => {
  it("a run with a recording has a Replay button; one without, or while busy, does not work", () => {
    let m = applyEvent(emptyClaude(), { kind: "run", n: 1 });
    m = applyEvent(m, { kind: "run_result", ok: true, message: "ok", output: "RESULT: ok", trace: "runs/task_001-1.json" });
    const messages: Message[] = [m, { role: "run", path: "programs/task_002.py", ok: false, output: "x", trace: null }];
    const onReplay = vi.fn();
    const { rerender } = render(<Chat messages={messages} live={null} running={false} disabled={false} onSend={vi.fn()} onReplay={onReplay} />);
    const buttons = screen.getAllByRole("button", { name: /Replay/ });
    expect(buttons).toHaveLength(1);                               // the manual run has no recording
    fireEvent.click(buttons[0]);
    expect(onReplay).toHaveBeenCalledWith("runs/task_001-1.json");
    rerender(<Chat messages={messages} live={null} running={true} disabled={false} onSend={vi.fn()} onReplay={onReplay} />);
    expect((screen.getByRole("button", { name: /Replay/ }) as HTMLButtonElement).disabled).toBe(true);
  });
});

describe("Replay controls", () => {
  const viewer = () => ({ setBodyPoses: vi.fn(), setSceneObjects: vi.fn(), setFollow: vi.fn() }) as unknown as
    Viewer & { setBodyPoses: ReturnType<typeof vi.fn>; setFollow: ReturnType<typeof vi.fn> };

  it("a mobile robot's replay (recorded in map) has the camera ride along with its base", () => {
    const v = viewer();
    const base = { id: "chassis", name: "chassis", body_ids: ["chassis_b"], density_kg_m3: 1000, mass_override_kg: null, accept_approximate_mass: false };
    const { unmount } = render(<Replay trace={{ ...trace, base: "map" }} viewer={v} links={[base]} joints={[]} rootLinkId="chassis" onStop={vi.fn()} />);
    expect(v.setFollow).toHaveBeenCalledWith("chassis_b");
    unmount();
    expect(v.setFollow).toHaveBeenLastCalledWith(null);
    render(<Replay trace={trace} viewer={v} links={[base]} joints={[]} rootLinkId="chassis" onStop={vi.fn()} />);
    expect(v.setFollow).toHaveBeenCalledTimes(2);           // a bolted-down robot's replay: no following
  });

  it("scrubbing moves the robot to that moment; the run's result is shown; Stop hands back to live", () => {
    const v = viewer();
    const onStop = vi.fn();
    render(<Replay trace={trace} viewer={v} links={[]} joints={[]} rootLinkId={null} onStop={onStop} />);
    expect(screen.getByText(/failed: Unreachable: too far/)).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "Pause" }));
    const calls = v.setBodyPoses.mock.calls.length;
    act(() => { fireEvent.change(screen.getByLabelText("Replay time"), { target: { value: "1.5" } }); });
    expect(v.setBodyPoses.mock.calls.length).toBeGreaterThan(calls);
    expect(screen.getByText("1.5 / 2.0 s")).toBeTruthy();
    fireEvent.click(screen.getByRole("button", { name: "Stop replay" }));
    expect(onStop).toHaveBeenCalled();
  });
});
