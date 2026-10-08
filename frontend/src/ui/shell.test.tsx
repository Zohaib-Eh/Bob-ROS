import { fireEvent, render, screen } from "@testing-library/react";
import { useReducer, useState } from "react";
import { describe, expect, it, vi } from "vitest";
import { historyReducer, initialHistory } from "../model/history";
import { boxBody } from "../model/testBodies";
import PartsTree from "./PartsTree";
import type { Selection } from "./selection";
import StatusBar from "./StatusBar";

function TreeHarness() {
  const [h, dispatch] = useReducer(historyReducer, undefined, () => historyReducer(initialHistory(), {
    type: "import", warnings: [{ code: "unaccounted_faces", message: "2 faces lost", fixes: ["reexport"] }],
    bodies: [boxBody("base", [0, 0, 0], [1, 1, 1]), boxBody("arm", [1, 0, 0], [2, 1, 1]),
      boxBody("screw", [0.5, 0.5, 1], [0.6, 0.6, 1.1], { status: "approximate" })] }));
  const [selection, setSelection] = useState<Selection>(null);
  return <PartsTree model={h.present} dispatch={dispatch} selection={selection} setSelection={setSelection} errors={{}} />;
}

describe("PartsTree", () => {
  it("shows file warnings, base marker and part status", () => {
    render(<TreeHarness />);
    expect(screen.getByText("2 faces lost")).toBeTruthy();
    expect(screen.getByTestId("part-screw").className).toMatch(/status-approximate/);
    expect(screen.getAllByText("⚓")).toHaveLength(1);
  });

  it("merges selected parts into one link", () => {
    render(<TreeHarness />);
    fireEvent.click(screen.getByTestId("part-base"));
    fireEvent.click(screen.getByTestId("part-arm"), { shiftKey: true });
    fireEvent.click(screen.getByTestId("merge-selected"));
    expect(screen.getAllByTestId(/^link-/)).toHaveLength(2);
  });

  it("merges small parts into their neighbours", () => {
    render(<TreeHarness />);
    fireEvent.click(screen.getByTestId("merge-small"));
    expect(screen.getAllByTestId(/^link-/)).toHaveLength(2);
  });
});

describe("StatusBar", () => {
  it("shows progress and cancel", () => {
    const onCancel = vi.fn();
    render(<StatusBar progress={{ done: 3, total: 9, running: true, paused: false }} onCancel={onCancel} onResume={vi.fn()} />);
    expect(screen.getByText("Checking joints: 3 of 9")).toBeTruthy();
    fireEvent.click(screen.getByText("Cancel"));
    expect(onCancel).toHaveBeenCalled();
  });
});
