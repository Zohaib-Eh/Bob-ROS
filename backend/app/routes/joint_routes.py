from fastapi import APIRouter
from pydantic import BaseModel

from app.joints.face_matching import bodies_in_contact, find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.models import Body, Joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body
    body_b: Body
    parent_link_id: str
    child_link_id: str


@router.post("/detect-joints", response_model=Joint | None)
def detect_joints(request: DetectJointsRequest) -> Joint | None:
    if not bodies_in_contact(request.body_a, request.body_b):
        return None  # no contact -> no suggestion (spec 9), not a guessed "fixed"
    pairs = find_matching_face_pairs(request.body_a, request.body_b)
    return classify_joint(pairs, request.parent_link_id, request.child_link_id)
