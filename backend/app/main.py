from fastapi import FastAPI

from app.program import routes as program_routes
from app.routes import export_routes, import_routes, joint_routes, link_routes, replace_routes, sim_routes

app = FastAPI(title="cad-to-urdf")
app.include_router(import_routes.router)
app.include_router(joint_routes.router)
app.include_router(link_routes.router)
app.include_router(export_routes.router)
app.include_router(replace_routes.router)
app.include_router(program_routes.router)
app.include_router(sim_routes.router)


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
