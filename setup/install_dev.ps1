# ============================================================================
# setup/install_dev.ps1 — DEV MACHINE setup (Windows laptop, NOT the Jetson)
#
# Windows twin of install_dev.sh: venv + deps + verification. No ROS, no Docker,
# no hardware required. Node code is built and run on the Jetson, inside the
# `asv` container.
#
# Usage:   powershell -ExecutionPolicy Bypass -File setup\install_dev.ps1
#          ... -NoVerify      # skip the verification run
# ============================================================================
param([switch]$NoVerify)
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")   # repo root

Write-Host "== [1/3] Python virtual environment (.venv) =="
python -m venv .venv
& .\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip

Write-Host "== [2/3] Dependencies (matches CI) =="
pip install pyyaml
# Optional: pip install pymavlink              (param_guard.py --live)
# Optional: pip install opencv-python numpy    (oak_view.py on a raw topic)

if (-not $NoVerify) {
    Write-Host "== [3/3] Verify: config + param baseline guards =="
    python tools\scripts\check_config.py
    if ($LASTEXITCODE -ne 0) { throw "config guards failed" }
} else {
    Write-Host "== [3/3] skipped (-NoVerify) =="
}

Write-Host ""
Write-Host "Done. Activate with:  .\.venv\Scripts\Activate.ps1"
Write-Host "Next steps: docs/SETUP_GUIDE.md."
