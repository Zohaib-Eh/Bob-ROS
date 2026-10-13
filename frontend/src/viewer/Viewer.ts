import {
  Box3, BoxGeometry, BufferAttribute, BufferGeometry, CylinderGeometry, DirectionalLight, EdgesGeometry, GridHelper, Group,
  HemisphereLight, LineBasicMaterial, LineSegments, Material, Matrix4, Mesh, MeshBasicMaterial, MeshStandardMaterial, Object3D,
  PerspectiveCamera, Raycaster, Scene, Sphere, Vector2, Vector3, WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { Body, Joint, Vec3 } from "../api/types";
import type { SceneObject } from "../ros/liveRobot";
import { bodyArrays, isClick } from "./meshData";
import {
  angleAroundAxis, buildGlyph, buildLooseLine, buildSnap, closestParamOnAxis, disposeTree, rayPlaneHit, unwrapStep,
  type JointGlyph,
} from "./overlays";
import { fitDistance, viewDirection, type ViewName } from "./viewMath";

export type Hit = { bodyId: string; triangle: number } | null;
type PickCb = (hit: Hit, additive: boolean) => void;
type LimitCb = (which: "lower" | "upper", value: number) => void;
export type DriveSpec = { joint: Joint; bodyIds: string[]; value: number; onChange: (v: number) => void };

export const PART_COLOR = 0xb9bdc3;
const SELECT_EMISSIVE = 0x1d4ed8;
const HOVER_EMISSIVE = 0x475569;
const APPROX_COLOR = 0xd97706;
const WARNING_COLOR = 0xdc2626;

export class Viewer {
  private renderer: WebGLRenderer;
  private scene = new Scene();
  private camera = new PerspectiveCamera(45, 1, 0.001, 1000);
  private controls: OrbitControls;
  private meshes = new Map<string, Mesh>();
  private edgeMaterial = new LineBasicMaterial({ color: 0x3f4349, transparent: true, opacity: 0.35 });
  private grid: GridHelper | null = null;
  private bounds = new Sphere(new Vector3(), 0.5);
  private pickCb: PickCb = () => {};
  private hoverCb: ((hit: Hit) => void) | null = null;
  private cameraCbs = new Set<() => void>();
  private frame = 0;
  private dirty = true;
  private framed = false;
  private observer: ResizeObserver;
  private selection: string[] = [];
  private hovered: string | null = null;
  private listHover: string[] = [];
  private tints: Record<string, number> = {};
  private preview: { ids: string[]; m: Matrix4 | null } = { ids: [], m: null };
  private ignored = new Set<string>();
  private down: { x: number; y: number } | null = null;
  private pendingHover: PointerEvent | null = null;
  private overlay = new Group();
  private glyphs: JointGlyph[] = [];
  private loose: [Vec3, Vec3][] = [];
  private snap: { origin: Vec3; axis: Vec3 } | null = null;
  private approximate: string[] = [];
  private limitCb: LimitCb | null = null;
  private dragging: { which: "lower" | "upper"; joint: Joint } | null = null;
  private drive: DriveSpec | null = null;
  private driving: { last: number } | null = null;
  private warning: string[] = [];

  constructor(private container: HTMLElement) {
    this.renderer = new WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    this.scene.add(new HemisphereLight(0xffffff, 0x8a8f99, 0.9));
    const key = new DirectionalLight(0xffffff, 1.1);
    key.position.set(3, -4, 6);
    const fill = new DirectionalLight(0xffffff, 0.35);
    fill.position.set(-4, 3, 2);
    this.scene.add(key, fill, this.overlay);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2, -2, 1.5);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.addEventListener("change", this.requestRender);
    const el = this.renderer.domElement;
    el.addEventListener("pointerdown", this.onDown);
    el.addEventListener("pointermove", this.onMove);
    el.addEventListener("pointerup", this.onUp);
    this.observer = new ResizeObserver(() => this.resize());
    this.observer.observe(container);
    this.resize();
    const loop = () => {
      if (this.pendingHover) {
        const e = this.pendingHover;
        this.pendingHover = null;
        this.hover(e);
      }
      if (this.dirty) {
        this.dirty = false;
        this.renderer.render(this.scene, this.camera);
        this.cameraCbs.forEach((cb) => cb());
      }
      this.frame = requestAnimationFrame(loop);
    };
    loop();
  }

  /** Draw on the next frame. Nothing is drawn while nothing changes. */
  requestRender = () => { this.dirty = true; };

  private resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  setBodies(bodies: Body[], ignored: Set<string>) {
    this.ignored = ignored;
    const keep = new Set(bodies.map((b) => b.id));
    for (const [id, m] of this.meshes) {
      if (!keep.has(id)) { this.scene.remove(m); this.free(m); this.meshes.delete(id); }
    }
    for (const b of bodies) {
      const old = this.meshes.get(b.id);
      if (old && old.userData.source === b) continue;
      if (old && old.userData.source.vertices === b.vertices && old.userData.source.triangles === b.triangles) {
        old.userData.source = b;   // renamed, same geometry: keep the mesh
        continue;
      }
      if (old) { this.scene.remove(old); this.free(old); }
      const { positions, indices } = bodyArrays(b);
      const g = new BufferGeometry();
      g.setAttribute("position", new BufferAttribute(positions, 3));
      g.setIndex(new BufferAttribute(indices, 1));
      g.computeVertexNormals();
      const mesh = new Mesh(g, new MeshStandardMaterial({ color: PART_COLOR, roughness: 0.55, metalness: 0.1 }));
      const edges = new LineSegments(new EdgesGeometry(g, 25), this.edgeMaterial);
      edges.name = "edges";
      mesh.add(edges);
      mesh.userData = { bodyId: b.id, source: b };
      this.scene.add(mesh);
      this.meshes.set(b.id, mesh);
    }
    this.computeBounds();
    if (!this.framed && this.meshes.size) { this.setView("home"); this.framed = true; }
    this.applyLooks();
    this.setPreview(this.preview.ids, this.preview.m);
    this.applyApproximate();
    this.rebuildOverlay();
  }

  private free(m: Mesh) {
    m.geometry.dispose();
    (m.material as Material).dispose();
    for (const c of m.children) {
      if (c.name === "edges") (c as LineSegments).geometry.dispose();
      if (c.name === "approx") ((c as Mesh).material as Material).dispose();
    }
  }

  private computeBounds() {
    const b = new Box3();
    for (const m of this.meshes.values()) if (!this.ignored.has(m.userData.bodyId)) b.expandByObject(m);
    if (b.isEmpty()) return;
    b.getBoundingSphere(this.bounds);
    if (this.grid) {
      this.scene.remove(this.grid);
      this.grid.geometry.dispose();
      (this.grid.material as Material).dispose();
    }
    this.grid = new GridHelper(this.bounds.radius * 4, 20, 0x9aa1ab, 0xc4c9d0);
    this.grid.rotateX(Math.PI / 2);
    this.grid.position.set(this.bounds.center.x, this.bounds.center.y, b.min.z);
    const gm = this.grid.material as Material;
    gm.transparent = true;
    gm.opacity = 0.5;
    this.scene.add(this.grid);
  }

  private applyLooks() {
    for (const [id, m] of this.meshes) {
      const mat = m.material as MeshStandardMaterial;
      mat.color.setHex(this.warning.includes(id) ? WARNING_COLOR : this.tints[id] ?? PART_COLOR);
      const ignored = this.ignored.has(id);
      mat.transparent = ignored;
      mat.opacity = ignored ? 0.15 : 1;
      mat.depthWrite = !ignored;
      const sel = this.selection.includes(id);
      mat.emissive.setHex(sel ? SELECT_EMISSIVE : id === this.hovered || this.listHover.includes(id) ? HOVER_EMISSIVE : 0x000000);
      mat.emissiveIntensity = sel ? 0.45 : 0.25;
    }
    this.requestRender();
  }

  setSelection(bodyIds: string[]) { this.selection = bodyIds; this.applyLooks(); }

  setTints(t: Record<string, number>) { this.tints = t; this.applyLooks(); }

  /** Highlight parts hovered in a list outside the 3D view. */
  setHovered(bodyIds: string[]) { this.listHover = bodyIds; this.applyLooks(); }

  setPreview(bodyIds: string[], m: Matrix4 | null) {
    this.preview = { ids: bodyIds, m };
    for (const [id, mesh] of this.meshes) {
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(m && bodyIds.includes(id) ? m : new Matrix4());
      mesh.matrixWorldNeedsUpdate = true;
    }
    this.requestRender();
  }

  private objects = new Group();

  /** Per-body poses (e.g. a live robot); bodies not listed stay at their CAD pose. */
  setBodyPoses(poses: Record<string, Matrix4>) {
    for (const [id, mesh] of this.meshes) {
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(poses[id] ?? new Matrix4());
      mesh.matrixWorldNeedsUpdate = true;
    }
    this.requestRender();
  }

  setSceneObjects(objs: SceneObject[]) {
    if (!this.objects.parent) this.scene.add(this.objects);
    for (const c of [...this.objects.children] as Mesh[]) {
      this.objects.remove(c); c.geometry.dispose(); (c.material as Material).dispose();
    }
    for (const o of objs) {
      const geo = o.shape === "box" ? new BoxGeometry(o.dims[0], o.dims[1], o.dims[2])
        : new CylinderGeometry(o.dims[1], o.dims[1], o.dims[0], 32).rotateX(Math.PI / 2); // ROS cylinders stand on z
      const mesh = new Mesh(geo, new MeshStandardMaterial({ color: o.attached ? 0xf59e0b : 0x60a5fa }));
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(o.matrix);
      this.objects.add(mesh);
    }
    this.requestRender();
  }

  setApproximate(bodyIds: string[]) { this.approximate = bodyIds; this.applyApproximate(); }

  private applyApproximate() {
    for (const m of this.meshes.values()) {
      const w = m.getObjectByName("approx") as Mesh | undefined;
      if (w) { m.remove(w); (w.material as Material).dispose(); }
    }
    for (const id of this.approximate) {
      const m = this.meshes.get(id);
      if (!m) continue;
      // shares the part's geometry, so it follows the part during previews
      const w = new Mesh(m.geometry, new MeshBasicMaterial({ color: APPROX_COLOR, wireframe: true, transparent: true, opacity: 0.08 }));
      w.name = "approx";
      m.add(w);
    }
    this.requestRender();
  }

  setJointGlyphs(g: JointGlyph[]) { this.glyphs = g; this.rebuildOverlay(); }
  setLooseLines(l: [Vec3, Vec3][]) { this.loose = l; this.rebuildOverlay(); }
  setSnap(s: { origin: Vec3; axis: Vec3 } | null) { this.snap = s; this.rebuildOverlay(); }
  onLimitDrag(cb: LimitCb | null) { this.limitCb = cb; }

  onDrive(d: DriveSpec | null) {
    this.drive = d;
    if (!d && this.driving) { this.driving = null; this.controls.enabled = true; }
  }

  setWarning(bodyIds: string[]) { this.warning = bodyIds; this.applyLooks(); }

  /** The pointer's position along the joint: angle around the axis, or distance along it. */
  private jointParam(e: PointerEvent, j: Joint): number | null {
    if (!j.axis) return null;
    const r = this.ray(e).ray;
    const ro = r.origin.toArray() as Vec3, rd = r.direction.toArray() as Vec3;
    if (j.type === "prismatic") return closestParamOnAxis(ro, rd, j.origin_xyz, j.axis);
    const p = rayPlaneHit(ro, rd, j.origin_xyz, j.axis);
    return p ? angleAroundAxis(p, j.origin_xyz, j.axis) : null;
  }

  private rebuildOverlay() {
    disposeTree(this.overlay);
    this.overlay.clear();
    const size = Math.max(this.bounds.radius * 0.2, 1e-3);
    for (const g of this.glyphs) this.overlay.add(buildGlyph(g, size));
    for (const [a, b] of this.loose) this.overlay.add(buildLooseLine(a, b, size));
    if (this.snap) this.overlay.add(buildSnap(this.snap, size));
    this.overlay.traverse((c) => {
      c.renderOrder = 10;
      const mat = (c as Mesh).material as Material | undefined;
      if (mat) mat.depthTest = false;   // glyphs stay visible inside the parts
    });
    this.requestRender();
  }

  private drag(e: PointerEvent) {
    const { which, joint } = this.dragging!;
    const value = this.jointParam(e, joint);
    if (value !== null) this.limitCb?.(which, value);
  }

  setView(v: ViewName | "home") {
    this.computeBounds();
    const dir = new Vector3(...viewDirection(v === "home" ? "iso" : v));
    const d = fitDistance(this.bounds.radius, this.camera.fov);
    this.camera.position.copy(this.bounds.center).addScaledVector(dir, d);
    this.camera.near = d / 1000;
    this.camera.far = d * 100;
    this.camera.updateProjectionMatrix();
    this.controls.target.copy(this.bounds.center);
    this.controls.update();
    this.requestRender();
  }

  focus(bodyIds: string[]) {
    const b = new Box3();
    for (const id of bodyIds) { const m = this.meshes.get(id); if (m) b.expandByObject(m); }
    if (b.isEmpty()) { this.setView("home"); return; }
    const s = new Sphere();
    b.getBoundingSphere(s);
    const dir = this.camera.position.clone().sub(this.controls.target).normalize();
    this.controls.target.copy(s.center);
    this.camera.position.copy(s.center).addScaledVector(dir, fitDistance(s.radius, this.camera.fov));
    this.controls.update();
    this.requestRender();
  }

  screenPoint(p: Vec3): { x: number; y: number } | null {
    const v = new Vector3(...p).project(this.camera);
    if (v.z < -1 || v.z > 1) return null;
    const { clientWidth: w, clientHeight: h } = this.container;
    return { x: ((v.x + 1) / 2) * w, y: ((1 - v.y) / 2) * h };
  }

  onCameraChange(cb: () => void): () => void {
    this.cameraCbs.add(cb);
    return () => { this.cameraCbs.delete(cb); };
  }

  onPick(cb: PickCb) { this.pickCb = cb; }

  onHover(cb: ((hit: Hit) => void) | null) {
    this.hoverCb = cb;
    if (!cb && this.hovered) { this.hovered = null; this.applyLooks(); }
  }

  private ray(e: { clientX: number; clientY: number }): Raycaster {
    const r = this.renderer.domElement.getBoundingClientRect();
    const ray = new Raycaster();
    ray.setFromCamera(new Vector2(((e.clientX - r.left) / r.width) * 2 - 1,
      -((e.clientY - r.top) / r.height) * 2 + 1), this.camera);
    return ray;
  }

  private hitBody(ray: Raycaster): Hit {
    const meshes = [...this.meshes.values()].filter((m) => !this.ignored.has(m.userData.bodyId));
    const i = ray.intersectObjects(meshes, false)[0];
    return i ? { bodyId: i.object.userData.bodyId, triangle: i.faceIndex ?? -1 } : null;
  }

  private hover(e: PointerEvent) {
    const hit = this.hitBody(this.ray(e));
    const id = hit?.bodyId ?? null;
    if (id !== this.hovered) { this.hovered = id; this.applyLooks(); }
    this.hoverCb?.(hit);
  }

  private onDown = (e: PointerEvent) => {
    this.down = { x: e.clientX, y: e.clientY };
    const glyph = this.glyphs.find((g) => g.state === "active");
    if (this.limitCb && glyph) {
      const handles: Object3D[] = [];
      this.overlay.traverse((c) => { if (c.userData.handle) handles.push(c); });
      const h = this.ray(e).intersectObjects(handles, false)[0];
      if (h) {
        this.dragging = { which: h.object.userData.handle, joint: glyph.joint };
        this.controls.enabled = false;
        return;
      }
    }
    if (this.drive) {
      const hit = this.hitBody(this.ray(e));
      const p = hit && this.drive.bodyIds.includes(hit.bodyId) ? this.jointParam(e, this.drive.joint) : null;
      if (p !== null) { this.driving = { last: p }; this.controls.enabled = false; }
    }
  };

  private onMove = (e: PointerEvent) => {
    if (this.dragging) { this.drag(e); return; }
    if (this.driving && this.drive) {
      const p = this.jointParam(e, this.drive.joint);
      if (p !== null) {
        const step = this.drive.joint.type === "prismatic" ? p - this.driving.last : unwrapStep(this.driving.last, p);
        this.driving.last = p;
        this.drive.value += step;
        this.drive.onChange(this.drive.value);
      }
      return;
    }
    if (this.hoverCb && !this.down) this.pendingHover = e;
  };

  private onUp = (e: PointerEvent) => {
    const down = this.down;
    this.down = null;
    if (this.dragging || this.driving) { this.dragging = null; this.driving = null; this.controls.enabled = true; return; }
    if (!down || !isClick(down, { x: e.clientX, y: e.clientY })) return;
    this.pickCb(this.hitBody(this.ray(e)), e.shiftKey);
  };

  dispose() {
    cancelAnimationFrame(this.frame);
    const el = this.renderer.domElement;
    el.removeEventListener("pointerdown", this.onDown);
    el.removeEventListener("pointermove", this.onMove);
    el.removeEventListener("pointerup", this.onUp);
    this.observer.disconnect();
    this.cameraCbs.clear();
    this.controls.dispose();
    for (const m of this.meshes.values()) this.free(m);
    this.edgeMaterial.dispose();
    if (this.grid) { this.grid.geometry.dispose(); (this.grid.material as Material).dispose(); }
    disposeTree(this.overlay);
    this.renderer.forceContextLoss();
    this.renderer.dispose();
    el.remove();
  }
}
