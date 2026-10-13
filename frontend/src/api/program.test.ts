import { afterEach, describe, expect, it, vi } from "vitest";
import { parseSse, runTask, type ProgramEvent } from "./program";

afterEach(() => vi.restoreAllMocks());

describe("program api", () => {
  it("parses SSE events split across chunks exactly once", async () => {
    const text = 'data: {"kind":"text","text":"a"}\n\ndata: {"kind":"run","n":1}\n\ndata: {"kind":"done","summary":"s","runs":1,"ok":true,"session_id":"x","program":null,"code":null}\n\n';
    const chunks = [text.slice(0, 17), text.slice(17, 50), text.slice(50)];
    const body = new ReadableStream({ start(c) { for (const k of chunks) c.enqueue(new TextEncoder().encode(k)); c.close(); } });
    vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(body, { status: 200 }));
    const got: ProgramEvent[] = [];
    await runTask("go", null, (e) => got.push(e));
    expect(got.map((e) => e.kind)).toEqual(["text", "run", "done"]);
  });

  it("keeps an unfinished event for the next chunk", () => {
    const { events, rest } = parseSse('data: {"kind":"run","n":1}\n\ndata: {"kind"');
    expect(events).toEqual([{ kind: "run", n: 1 }]);
    expect(rest).toBe('data: {"kind"');
  });

  it("parses CRLF-framed events", () => {
    const { events } = parseSse('data: {"kind":"run","n":1}\r\n\r\ndata: {"kind":"run","n":2}\r\n\r\n');
    expect(events).toEqual([{ kind: "run", n: 1 }, { kind: "run", n: 2 }]);
  });

  it("skips a bad frame between good ones", () => {
    const { events } = parseSse('data: {"kind":"run","n":1}\n\ndata: {oops\n\ndata: {"kind":"run","n":2}\n\n');
    expect(events).toEqual([{ kind: "run", n: 1 }, { kind: "run", n: 2 }]);
  });

  it("delivers a final event without a trailing blank line", async () => {
    const body = new ReadableStream({ start(c) { c.enqueue(new TextEncoder().encode('data: {"kind":"run","n":7}')); c.close(); } });
    vi.spyOn(globalThis, "fetch").mockResolvedValue(new Response(body, { status: 200 }));
    const got: ProgramEvent[] = [];
    await runTask("go", null, (e) => got.push(e));
    expect(got).toEqual([{ kind: "run", n: 7 }]);
  });
});
