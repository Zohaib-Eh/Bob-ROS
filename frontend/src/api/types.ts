export type Vec3 = [number, number, number];
export type Status = "ok" | "repaired" | "approximate" | "failed";
export type Fix = "set_mass" | "replace" | "ignore" | "reexport";

export interface PartWarning { code: string; message: string; fixes: Fix[] }

export interface Face {
  kind: "plane" | "cylinder" | "sphere" | "unclassified";
  origin: "brep" | "segmented";
  normal: Vec3 | null;
  axis: Vec3 | null;
  center: Vec3;
  radius: number | null;
  residual: number;
  triangle_indices: number[];
}

export interface Body {
  id: string;
  name: string;
  vertices: Vec3[];
  triangles: [number, number, number][];
  faces: Face[];
  status: Status;
  warnings: PartWarning[];
  source?: string;   // the CAD file it was imported from (unique per import); absent in older projects
}

export interface Link {
  id: string;
  name: string;
  body_ids: string[];
  density_kg_m3: number;
  mass_override_kg: number | null;
  accept_approximate_mass: boolean;
}

export type JointType =
  | "revolute" | "continuous" | "prismatic" | "cylindrical" | "planar" | "spherical" | "fixed";

export interface Joint {
  id: string;
  parent_link_id: string;
  child_link_id: string;
  type: JointType;
  origin_xyz: Vec3;
  origin_rpy: Vec3;
  axis: Vec3 | null;
  limit_lower: number | null;
  limit_upper: number | null;
  source: "suggested" | "confirmed" | "manual";
  ambiguous?: boolean;
  max_effort?: number | null;   // N·m (N for prismatic); null/missing = auto
}

export interface Evidence {
  pairs: { kind: string; radius: number | null }[];
  candidate_count: number;
  tolerance: number;
}

export interface DetectResponse {
  joint: Joint | null;
  reason: "no_overlap" | "no_contact_faces" | null;
  evidence: Evidence | null;
}

export interface ImportResponse { bodies: Body[]; applied_scale: number; warnings: PartWarning[] }

export interface LinkMass { mass_kg: number; center_of_mass: Vec3; inertia: number[] }

export interface SweepResult {
  lower: number | null;
  upper: number | null;
  free: boolean;
  blocked_by: { lower: string | null; upper: string | null };
  at_rest_overlap: string[];
}
