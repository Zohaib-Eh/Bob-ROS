import { act, fireEvent, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";
import * as program from "../../api/program";
import { boxBody } from "../../model/testBodies";
import { Harness, imported } from "../testing";
import ProgramStep from "./ProgramStep";

vi.mock("../../ros/rosbridge", () => ({
  RosBridge: class { ready = Promise.resolve(); subscribe() {} call() { return Promise.resolve({ scene: {} }); } close() {} },
}));
afterEach(() => vi.restoreAllMocks());

const ui = () => render(<Harness start={imported([boxBody("base", [0, 0, 0], [1, 1, 1])])} step="program"
  ui={(ctx) => <ProgramStep ctx={ctx} />} />);

describe("ProgramStep", () => {
  it("starts a session, runs a task and shows the result, runs used and the program", async () => {
    vi.spyOn(program, "startSession").mockResolvedValue({ state: "starting", robot: "bot" });
    vi.spyOn(program, "sessionStatus").mockResolvedValue({ state: "ready", robot: "bot" });
    vi.spyOn(program, "runTask").mockImplementation(async (_t, _r, on) => {
      on({ kind: "text", text: "Writing the program" });
      on({ kind: "run", n: 1 });
      on({ kind: "run_result", ok: false, message: "failed: Unreachable: too far" });
      on({ kind: "run", n: 2 });
      on({ kind: "run_result", ok: true, message: "ok" });
      on({ kind: "done", summary: "Moves the tool.\nRun ok.", runs: 2, ok: true, session_id: "s1",
           program: "programs/task_001.py", code: "from robot import Robot" });
    });
    const rerun = vi.spyOn(program, "rerun").mockResolvedValue({ ok: true, output: "RESULT: ok" });
    ui();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
    await screen.findByText(/ROS ready/i);
    fireEvent.change(screen.getByLabelText(/task/i), { target: { value: "move the tool" } });
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /^send$/i })); });
    expect(screen.getByText(/worked on run 2\/3/i)).toBeTruthy();
    expect(screen.getByText(/Moves the tool\./)).toBeTruthy();
    expect(screen.getByText("from robot import Robot")).toBeTruthy();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /run again/i })); });
    expect(rerun).toHaveBeenCalledWith("programs/task_001.py");
  });

  it("shows backend errors in the chat", async () => {
    vi.spyOn(program, "startSession").mockRejectedValue(new Error("Start Docker Desktop."));
    ui();
    await act(async () => { fireEvent.click(screen.getByRole("button", { name: /start ros/i })); });
    expect(await screen.findByText("Start Docker Desktop.")).toBeTruthy();
  });
});
