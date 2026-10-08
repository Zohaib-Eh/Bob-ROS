from typing import Literal

from fastapi import APIRouter
from pydantic import BaseModel

from app.joints.face_matching import (
    _PLANE_DISTANCE_TOLERANCE_DEFAULT, bodies_in_contact, find_matching_face_pairs,
)
from app.joints.ipv_img import classify_joint
from app.models import Body, Joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body
    body_b: Body
    parent_link_id: str
    child_link_id: str


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
    if not bodies_in_contact(request.body_a, request.body_b, tol):
        return DetectJointsResponse(joint=None, reason="no_overlap", evidence=None)
    pairs = find_matching_face_pairs(request.body_a, request.body_b, tol)
    if not pairs:  # spec §9: explicit "no suggestion", not a guessed "fixed"
        return DetectJointsResponse(joint=None, reason="no_contact_faces", evidence=None)
    joint = classify_joint(pairs, request.parent_link_id, request.child_link_id)
    evidence = Evidence(
        pairs=[PairEvidence(kind=a.kind, radius=a.radius) for a, _ in pairs],
        candidate_count=len(pairs), tolerance=tol,
    )
    return DetectJointsResponse(joint=joint, reason=None, evidence=evidence)
