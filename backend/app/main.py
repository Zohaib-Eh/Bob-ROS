from fastapi import FastAPI

app = FastAPI(title="cad-to-urdf")

@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}
