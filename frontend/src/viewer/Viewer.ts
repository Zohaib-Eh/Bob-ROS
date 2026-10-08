import {
  AmbientLight, ArrowHelper, Box3, BufferAttribute, BufferGeometry, Color, DirectionalLight, Group, Matrix4,
  Mesh, MeshStandardMaterial, Object3D, PerspectiveCamera, Raycaster, Scene, Vector2, Vector3, WebGLRenderer,
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import type { Body, Joint, Vec3 } from "../api/types";
import { bodyArrays, statusColor } from "./meshData";

type PickCb = (hit: { bodyId: string; triangle: number } | null, additive: boolean) => void;

export class Viewer {
  private renderer: WebGLRenderer;
  private scene = new Scene();
  private camera = new PerspectiveCamera(45, 1, 0.001, 1000);
  private controls: OrbitControls;
  private meshes = new Map<string, Mesh>();
  private joints = new Group();
  private gizmo: TransformControls;
  private gizmoTarget = new Object3D();
  private pickCb: PickCb = () => {};
  private frame = 0;
  private framed = false;

  constructor(private container: HTMLElement) {
    this.renderer = new WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);
    this.scene.background = new Color(0xf4f6f8);
    this.scene.add(new AmbientLight(0xffffff, 0.6));
    const sun = new DirectionalLight(0xffffff, 0.9);
    sun.position.set(3, 5, 4);
    this.scene.add(sun, this.joints, this.gizmoTarget);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2, -2, 1.5);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.addEventListener("dragging-changed", (e) => { this.controls.enabled = !e.value; });
    this.renderer.domElement.addEventListener("pointerdown", this.onPointer);
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    const loop = () => { this.renderer.render(this.scene, this.camera); this.frame = requestAnimationFrame(loop); };
    loop();
  }

  private resize() {
    const { clientWidth: w, clientHeight: h } = this.container;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
  }

  setBodies(bodies: Body[], ignored: Set<string>) {
    const keep = new Set(bodies.map((b) => b.id));
    for (const [id, m] of this.meshes) {
      if (!keep.has(id)) { this.scene.remove(m); m.geometry.dispose(); this.meshes.delete(id); }
    }
    for (const b of bodies) {
      let mesh = this.meshes.get(b.id);
      if (!mesh || mesh.userData.source !== b) {
        if (mesh) { this.scene.remove(mesh); mesh.geometry.dispose(); }
        const { positions, indices } = bodyArrays(b);
        const g = new BufferGeometry();
        g.setAttribute("position", new BufferAttribute(positions, 3));
        g.setIndex(new BufferAttribute(indices, 1));
        g.computeVertexNormals();
        mesh = new Mesh(g, new MeshStandardMaterial({ color: statusColor(b.status) }));
        mesh.userData = { bodyId: b.id, source: b };
        this.scene.add(mesh);
        this.meshes.set(b.id, mesh);
      }
      const mat = mesh.material as MeshStandardMaterial;
      mat.color.setHex(statusColor(b.status));
      mat.transparent = ignored.has(b.id);
      mat.opacity = ignored.has(b.id) ? 0.2 : 1;
    }
    if (!this.framed && bodies.length) this.frameAll();
  }

  private frameAll() {
    const size = new Vector3(), center = new Vector3();
    const b = new Box3();
    for (const m of this.meshes.values()) b.expandByObject(m);
    b.getSize(size); b.getCenter(center);
    const d = Math.max(size.x, size.y, size.z) * 1.8 || 1;
    this.camera.position.copy(center).add(new Vector3(d, -d, d * 0.7));
    this.controls.target.copy(center);
    this.controls.update();
    this.framed = true;
  }

  setSelection(bodyIds: string[]) {
    for (const [id, m] of this.meshes) {
      (m.material as MeshStandardMaterial).emissive.setHex(bodyIds.includes(id) ? 0x2266cc : 0x000000);
    }
  }

  setJoints(items: { joint: Joint; dashed: boolean }[]) {
    this.joints.clear();
    for (const { joint, dashed } of items) {
      if (!joint.axis) continue;
      const arrow = new ArrowHelper(new Vector3(...joint.axis).normalize(), new Vector3(...joint.origin_xyz),
        0.1, dashed ? 0x999999 : 0x1565c0);
      this.joints.add(arrow);
    }
  }

  setPreview(bodyIds: string[], m: Matrix4 | null) {
    for (const [id, mesh] of this.meshes) {
      mesh.matrixAutoUpdate = false;
      mesh.matrix.copy(m && bodyIds.includes(id) ? m : new Matrix4());
      mesh.matrixWorldNeedsUpdate = true;
    }
  }

  onPick(cb: PickCb) { this.pickCb = cb; }

  private onPointer = (e: PointerEvent) => {
    if (this.gizmo.dragging) return;
    const r = this.renderer.domElement.getBoundingClientRect();
    const ndc = new Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
    const ray = new Raycaster();
    ray.setFromCamera(ndc, this.camera);
    const hit = ray.intersectObjects([...this.meshes.values()], false)[0];
    this.pickCb(hit ? { bodyId: hit.object.userData.bodyId, triangle: hit.faceIndex ?? -1 } : null, e.shiftKey);
  };

  attachGizmo(origin: Vec3, axis: Vec3, onChange: (origin: Vec3, axis: Vec3) => void) {
    this.gizmoTarget.position.set(...origin);
    this.gizmoTarget.quaternion.setFromUnitVectors(new Vector3(0, 0, 1), new Vector3(...axis).normalize());
    this.gizmo.attach(this.gizmoTarget);
    this.scene.add(this.gizmo.getHelper());
    this.gizmo.removeEventListener("objectChange", this.gizmoChange);
    this.gizmoChange = () => {
      const p = this.gizmoTarget.position;
      const a = new Vector3(0, 0, 1).applyQuaternion(this.gizmoTarget.quaternion);
      onChange([p.x, p.y, p.z], [a.x, a.y, a.z]);
    };
    this.gizmo.addEventListener("objectChange", this.gizmoChange);
  }
  private gizmoChange = () => {};

  setGizmoMode(mode: "translate" | "rotate") { this.gizmo.setMode(mode); }

  detachGizmo() {
    this.gizmo.detach();
    this.scene.remove(this.gizmo.getHelper());
  }

  dispose() {
    cancelAnimationFrame(this.frame);
    this.renderer.domElement.removeEventListener("pointerdown", this.onPointer);
    this.controls.dispose();
    this.gizmo.dispose();
    for (const m of this.meshes.values()) m.geometry.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
