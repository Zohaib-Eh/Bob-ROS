import json

from fastapi import APIRouter, HTTPException, Response
from pydantic import BaseModel

from app.export.urdf_writer import export_package, unused_body_warnings
from app.models import Body, Gripper, Joint, Link

router = APIRouter()


class ExportRequest(BaseModel):
    robot_name: str
    links: list[Link]
    joints: list[Joint]
    bodies_by_link: dict[str, list[Body]]
    gripper: Gripper | None = None


@router.post("/export")
def export(request: ExportRequest) -> Response:
    try:
        data = export_package(
            request.robot_name, request.links, request.joints, request.bodies_by_link, gripper=request.gripper
        )
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    warnings = unused_body_warnings(request.links, request.bodies_by_link)
    return Response(
        content=data,
        media_type="application/zip",
        headers={
            "Content-Disposition": f'attachment; filename="{request.robot_name}.zip"',
            "X-Export-Warnings": json.dumps([w.model_dump() for w in warnings]),  # ensure_ascii
        },
    )
