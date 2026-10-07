from fastapi import APIRouter
from pydantic import BaseModel

from app.joints.face_matching import find_matching_face_pairs
from app.joints.ipv_img import classify_joint
from app.models import Body, Joint

router = APIRouter()


class DetectJointsRequest(BaseModel):
    body_a: Body
    body_b: Body
    parent_link_id: str
    child_link_id: str


@router.post("/detect-joints", response_model=Joint)
def detect_joints(request: DetectJointsRequest) -> Joint:
    pairs = find_matching_face_pairs(request.body_a, request.body_b)
    return classify_joint(pairs, request.parent_link_id, request.child_link_id)
