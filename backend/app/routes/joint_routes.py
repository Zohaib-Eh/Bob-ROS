from typing import Literal

from fastapi import APIRouter
from pydantic import BaseModel, model_validator

from app.importers.placement import combine_bodies
from app.joints.face_matching import (
    _PLANE_DISTANCE_TOLERANCE_DEFAULT, bodies_in_contact, find_matching_face_pairs,
)
from app.joints.ipv_img import classify_joint
from app.models import Body, Joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body | None = None
    body_b: Body | None = None
    bodies_a: list[Body] | None = None
    bodies_b: list[Body] | None = None
    parent_link_id: str
    child_link_id: str

    @model_validator(mode="after")
    def _one_form_per_side(self):
        if (self.body_a is None) == (self.bodies_a is None) or \
           (self.body_b is None) == (self.bodies_b is None):
            raise ValueError("give exactly one of body_a/bodies_a and one of body_b/bodies_b")
        return self

    def side(self, which: str) -> Body:
        single, many = (self.body_a, self.bodies_a) if which == "a" else (self.body_b, self.bodies_b)
        return single if single is not None else combine_bodies(many, which, which)


class PairEvidence(BaseModel):
    kind: str
    radius: float | None


class Evidence(BaseModel):
    pairs: list[PairEvidence]
    candidate_count: int
    tolerance: float


class DetectJointsResponse(BaseModel):
    joint: Joint | None
    reason: Literal["no_overlap", "no_contact_faces"] | None
    evidence: Evidence | None


@router.post("/detect-joints", response_model=DetectJointsResponse)
def detect_joints(request: DetectJointsRequest) -> DetectJointsResponse:
    tol = _PLANE_DISTANCE_TOLERANCE_DEFAULT
    a, b = request.side("a"), request.side("b")
    if not bodies_in_contact(a, b, tol):
        return DetectJointsResponse(joint=None, reason="no_overlap", evidence=None)
    pairs = find_matching_face_pairs(a, b, tol)
    if not pairs:  # spec §9: explicit "no suggestion", not a guessed "fixed"
        return DetectJointsResponse(joint=None, reason="no_contact_faces", evidence=None)
    joint = classify_joint(pairs, request.parent_link_id, request.child_link_id)
    evidence = Evidence(
        pairs=[PairEvidence(kind=a.kind, radius=a.radius) for a, _ in pairs],
        candidate_count=len(pairs), tolerance=tol,
    )
    return DetectJointsResponse(joint=joint, reason=None, evidence=evidence)
