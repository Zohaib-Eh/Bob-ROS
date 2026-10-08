import { Vector3 } from "three";
import { describe, expect, it } from "vitest";
import type { Joint } from "../api/types";
import { boxBody } from "../model/testBodies";
import { bodyArrays, faceAtTriangle, isClick, statusColor } from "./meshData";
import { jointMotion, previewRange, subtreeLinks } from "./preview";

const J = (over: Partial<Joint>): Joint => ({
  id: "j", parent_link_id: "a", child_link_id: "b", type: "revolute", origin_xyz: [1, 0, 0],
  origin_rpy: [0, 0, 0], axis: [0, 0, 1], limit_lower: -1, limit_upper: 2, source: "confirmed", ...over });

describe("meshData", () => {
  it("flattens vertices and triangles", () => {
    const { positions, indices } = bodyArrays(boxBody("b", [0, 0, 0], [1, 1, 1]));
    expect(positions.length).toBe(8 * 3);
    expect(indices.length).toBe(12 * 3);
  });
  it("colours by status", () => {
    expect(statusColor("approximate")).not.toBe(statusColor("ok"));
  });
  it("finds the face owning a triangle", () => {
    const b = boxBody("b", [0, 0, 0], [1, 1, 1], { faces: [{ kind: "plane", origin: "brep", normal: [0, 0, -1],
      axis: null, center: [0, 0, 0], radius: null, residual: 0, triangle_indices: [0, 1] }] });
    expect(faceAtTriangle(b, 1)?.normal).toEqual([0, 0, -1]);
    expect(faceAtTriangle(b, 5)).toBeNull();
  });
});

describe("preview", () => {
  it("rotates about the axis through the joint origin", () => {
    const p = new Vector3(2, 0, 0).applyMatrix4(jointMotion(J({}), Math.PI / 2));
    expect(p.x).toBeCloseTo(1); expect(p.y).toBeCloseTo(1); expect(p.z).toBeCloseTo(0);
  });
  it("translates along the axis for prismatic", () => {
    const p = new Vector3(0, 0, 0).applyMatrix4(jointMotion(J({ type: "prismatic", axis: [0, 2, 0] }), 0.5));
    expect(p.y).toBeCloseTo(0.5);
  });
  it("is identity for fixed joints", () => {
    const p = new Vector3(3, 4, 5).applyMatrix4(jointMotion(J({ type: "fixed" }), 1));
    expect([p.x, p.y, p.z]).toEqual([3, 4, 5]);
  });
  it("ranges: limits for revolute, full turn for continuous", () => {
    expect(previewRange(J({}))).toEqual([-1, 2]);
    expect(previewRange(J({ type: "continuous", limit_lower: null, limit_upper: null }))).toEqual([-Math.PI, Math.PI]);
  });
  it("subtree is the child link and everything below it", () => {
    const directed = [J({ id: "j1", parent_link_id: "a", child_link_id: "b" }),
      J({ id: "j2", parent_link_id: "b", child_link_id: "c" }), J({ id: "j3", parent_link_id: "a", child_link_id: "d" })];
    expect(subtreeLinks(directed, "j1").sort()).toEqual(["b", "c"]);
  });
});

describe("robustness", () => {
  it("subtree terminates on cycles and has no duplicates", () => {
    const directed = [J({ id: "j1", parent_link_id: "a", child_link_id: "b" }),
      J({ id: "j2", parent_link_id: "b", child_link_id: "a" }), J({ id: "j3", parent_link_id: "a", child_link_id: "b" })];
    expect(subtreeLinks(directed, "j1").sort()).toEqual(["a", "b"]);
  });
  it("distinguishes click from drag", () => {
    expect(isClick({ x: 10, y: 10 }, { x: 12, y: 11 })).toBe(true);
    expect(isClick({ x: 10, y: 10 }, { x: 20, y: 10 })).toBe(false);
  });
});
