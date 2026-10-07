from typing import Literal
from pydantic import BaseModel

Vec3 = tuple[float, float, float]

class Face(BaseModel):
    kind: Literal["plane", "cylinder", "sphere", "unclassified"]
    origin: Literal["brep", "segmented"]
    normal: Vec3 | None = None
    axis: Vec3 | None = None
    center: Vec3
    radius: float | None = None
    residual: float
    triangle_indices: list[int]

class Body(BaseModel):
    id: str
    name: str
    vertices: list[Vec3]
    triangles: list[tuple[int, int, int]]
    faces: list[Face] = []

class Link(BaseModel):
    id: str
    name: str
    body_ids: list[str]
    density_kg_m3: float
    mass_kg: float | None = None
    center_of_mass: Vec3 | None = None
    inertia: list[float] | None = None  # [ixx, iyy, izz, ixy, ixz, iyz]

class Joint(BaseModel):
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
