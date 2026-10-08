import { afterEach, describe, expect, it, vi } from "vitest";
import { ApiError, detectJoints, errorText, exportRobot, importFile } from "./client";

const ok = (body: unknown, headers: Record<string, string> = {}) =>
  new Response(typeof body === "string" ? body : JSON.stringify(body), { status: 200, headers });

afterEach(() => vi.restoreAllMocks());

describe("api client", () => {
  it("posts uploads to /api/import with the scale", async () => {
    const fetchMock = vi.spyOn(globalThis, "fetch").mockResolvedValue(
      ok({ bodies: [], applied_scale: 0.001, warnings: [] }));
    await importFile(new File(["x"], "part.stl"), 0.001);
    const [url, init] = fetchMock.mock.calls[0];
    expect(url).toBe("/api/import?scale=0.001");
    expect((init as RequestInit).method).toBe("POST");
    expect((init as RequestInit).body).toBeInstanceOf(FormData);
  });

  it("sends link body lists to /api/detect-joints", async () => {
    const fetchMock = vi.spyOn(globalThis, "fetch").mockResolvedValue(
      ok({ joint: null, reason: "no_overlap", evidence: null }));
    await detectJoints([], [], "a", "b");
    const body = JSON.parse((fetchMock.mock.calls[0][1] as RequestInit).body as string);
    expect(body).toEqual({ bodies_a: [], bodies_b: [], parent_link_id: "a", child_link_id: "b" });
  });

  it("raises ApiError with the server detail", async () => {
    vi.spyOn(globalThis, "fetch").mockResolvedValue(
      new Response(JSON.stringify({ detail: "part.stl: could not read mesh" }), { status: 422 }));
    await expect(importFile(new File(["x"], "part.stl"), 0.001)).rejects.toMatchObject(
      { status: 422, message: "part.stl: could not read mesh" });
    await expect(importFile(new File(["x"], "part.stl"), 0.001)).rejects.toBeInstanceOf(ApiError);
  });

  it("formats validation-error arrays", () => {
    expect(errorText([{ loc: ["body", "x"], msg: "field required" }])).toBe("body.x: field required");
  });

  it("returns the zip blob and parsed export warnings", async () => {
    vi.spyOn(globalThis, "fetch").mockResolvedValue(ok("ZIP", {
      "X-Export-Warnings": JSON.stringify([{ code: "unused_body", message: "m", fixes: [] }]) }));
    const out = await exportRobot("r", [], [], {});
    expect(await out.blob.text()).toBe("ZIP");
    expect(out.warnings[0].code).toBe("unused_body");
  });
});
