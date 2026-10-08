import { fireEvent, render, screen } from "@testing-library/react";
import { describe, expect, it } from "vitest";
import { deg, fmtLen, rad, shortName } from "./format";
import { GLOSSARY, type Term } from "./glossary";
import Info from "./Info";

describe("glossary", () => {
  it("every term has a title and a plain explanation", () => {
    for (const [key, [title, text]] of Object.entries(GLOSSARY)) {
      expect(title.length, key).toBeGreaterThan(2);
      expect(text.length, key).toBeGreaterThan(20);
    }
  });
  it("covers the terms the spec names", () => {
    const needed: Term[] = ["link", "base", "joint", "origin", "axis", "rigid", "revolute", "continuous", "slider",
      "planar", "cylindrical", "ball", "limits", "mass", "inertia", "estimated", "approximate", "failed", "urdf",
      "mesh", "scale", "name", "suggested"];
    for (const t of needed) expect(GLOSSARY[t]).toBeTruthy();
  });
});

describe("Info", () => {
  it("shows the explanation on focus and hides it on blur", () => {
    render(<Info term="revolute" />);
    const btn = screen.getByRole("button", { name: /What is Revolute/ });
    expect(screen.queryByRole("tooltip")).toBeNull();
    fireEvent.focus(btn);
    expect(screen.getByRole("tooltip").textContent).toMatch(/elbow/);
    fireEvent.blur(btn);
    expect(screen.queryByRole("tooltip")).toBeNull();
  });
});

describe("format", () => {
  it("shortens long names in the middle", () => {
    expect(shortName("short")).toBe("short");
    const s = shortName("IRB1300_7_12KG-1400-IP40-338203", 16);
    expect(s).toHaveLength(16);
    expect(s.startsWith("IRB1300_")).toBe(true);
    expect(s.endsWith("338203")).toBe(true);
    expect(s).toContain("…");
  });
  it("formats lengths in m or mm and converts angles", () => {
    expect(fmtLen(1.42)).toBe("1.42 m");
    expect(fmtLen(0.0254)).toBe("25 mm");
    expect(deg(Math.PI)).toBeCloseTo(180);
    expect(rad(90)).toBeCloseTo(Math.PI / 2);
  });
});
