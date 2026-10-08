import math
from typing import Literal
from pydantic import BaseModel, ConfigDict, model_validator

Vec3 = tuple[float, float, float]


class _Model(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)


def _check_direction(v: Vec3 | None, what: str) -> None:
    if v is not None and math.hypot(*v) < 1e-12:
        raise ValueError(f"{what} must have nonzero length")


Fix = Literal["set_mass", "replace", "ignore", "reexport"]
Status = Literal["ok", "repaired", "approximate", "failed"]
STATUS_ORDER = {"ok": 0, "repaired": 1, "approximate": 2, "failed": 3}


class PartWarning(_Model):
    code: str
    message: str
    fixes: list[Fix] = []


class Face(_Model):
    kind: Literal["plane", "cylinder", "sphere", "unclassified"]
    origin: Literal["brep", "segmented"]
    normal: Vec3 | None = None
    axis: Vec3 | None = None
    center: Vec3
    radius: float | None = None
    residual: float
    triangle_indices: list[int]

    @model_validator(mode="after")
    def _check_shape(self):
        if self.kind == "plane" and self.normal is None:
            raise ValueError("plane face requires normal")
        if self.kind == "cylinder" and self.axis is None:
            raise ValueError("cylinder face requires axis")
        if self.kind in ("cylinder", "sphere") and not (self.radius and self.radius > 0):
            raise ValueError(f"{self.kind} face requires radius > 0")
        _check_direction(self.normal, "normal")
        _check_direction(self.axis, "axis")
        return self


class Body(_Model):
    id: str
    name: str
    vertices: list[Vec3]
    triangles: list[tuple[int, int, int]]
    faces: list[Face] = []
    status: Status = "ok"
    warnings: list[PartWarning] = []

    @model_validator(mode="after")
    def _check_indices(self):
        n_v, n_t = len(self.vertices), len(self.triangles)
        if any(not 0 <= i < n_v for t in self.triangles for i in t):
            raise ValueError(f"triangle vertex index out of range [0, {n_v})")
        if any(not 0 <= i < n_t for f in self.faces for i in f.triangle_indices):
            raise ValueError(f"face triangle index out of range [0, {n_t})")
        return self


class Link(_Model):
    id: str
    name: str
    body_ids: list[str]
    density_kg_m3: float
    mass_kg: float | None = None
    center_of_mass: Vec3 | None = None
    inertia: list[float] | None = None  # [ixx, iyy, izz, ixy, ixz, iyz]
    mass_override_kg: float | None = None
    accept_approximate_mass: bool = False

    @model_validator(mode="after")
    def _check_override(self):
        if self.mass_override_kg is not None and not self.mass_override_kg > 0:
            raise ValueError("mass_override_kg must be > 0")
        return self


class Joint(_Model):
    id: str
    parent_link_id: str
    child_link_id: str
    type: Literal[
        "revolute", "continuous", "prismatic", "cylindrical",
        "planar", "spherical", "fixed",
    ]
    origin_xyz: Vec3
    origin_rpy: Vec3
    axis: Vec3 | None = None
    limit_lower: float | None = None
    limit_upper: float | None = None
    source: Literal["suggested", "confirmed", "manual"]
    ambiguous: bool = False

    @model_validator(mode="after")
    def _check(self):
        _check_direction(self.axis, "axis")
        if (self.limit_lower is not None and self.limit_upper is not None
                and self.limit_lower > self.limit_upper):
            raise ValueError("limit_lower must be <= limit_upper")
        return self
