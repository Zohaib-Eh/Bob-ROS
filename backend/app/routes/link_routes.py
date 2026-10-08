from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.export.urdf_writer import link_mass_properties
from app.models import Body, Link

router = APIRouter()


class LinkMassRequest(BaseModel):
    link: Link
    bodies: list[Body]


@router.post("/link-mass")
def link_mass(request: LinkMassRequest):
    try:
        m = link_mass_properties(request.link, request.bodies)
    except ValueError as exc:
        raise HTTPException(422, str(exc))
    return {"mass_kg": m.mass_kg, "center_of_mass": m.center_of_mass, "inertia": m.inertia}
