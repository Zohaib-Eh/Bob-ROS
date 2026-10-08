# Starts the backend (new window) and the frontend dev server (this window).
$root = Split-Path -Parent $PSScriptRoot
$py = Join-Path $env:USERPROFILE "miniforge3\envs\cad-to-urdf\python.exe"
Start-Process powershell -ArgumentList "-NoExit", "-Command", "cd '$root\backend'; `$env:OPENBLAS_NUM_THREADS='1'; & '$py' -m uvicorn app.main:app --port 8000"
Set-Location "$root\frontend"
npm run dev
