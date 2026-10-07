from fastapi import FastAPI

from app.routes import export_routes, import_routes, joint_routes

app = FastAPI(title="cad-to-urdf")
app.include_router(import_routes.router)
app.include_router(joint_routes.router)
app.include_router(export_routes.router)


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
