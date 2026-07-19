# ============================================================================
# setup/install_dev.ps1 — DEV MACHINE setup (Windows laptop, NOT the Jetson)
#
# Windows twin of install_dev.sh: venv + orchestrator/test deps + verification.
# No ROS, no Docker, no hardware required.
#
# Usage:   powershell -ExecutionPolicy Bypass -File setup\install_dev.ps1
#          ... -NoTest      # skip the verification run
# ============================================================================
param([switch]$NoTest)
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")   # repo root

Write-Host "== [1/3] Python virtual environment (.venv) =="
python -m venv .venv
& .\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip

Write-Host "== [2/3] Dependencies (orchestrator + tests; matches CI) =="
pip install pytest numpy pyyaml
# Optional: pip install anthropic     (only for --proposer llm runs)
# Optional: pip install ultralytics   (only for tools/training/)

if (-not $NoTest) {
    Write-Host "== [3/3] Verify: full test suite + Gate G0 smoke episode =="
    python -m pytest orchestrator/tests tests -q
    if ($LASTEXITCODE -ne 0) { throw "pytest failed" }
    $out = Join-Path $env:TEMP "ep_smoke.json"
    python orchestrator/run_episode.py `
        --scenario orchestrator/scenarios/mission1_transit.json `
        --backend kinematic --seed 0 --out $out
    if ($LASTEXITCODE -ne 0) { throw "G0 smoke episode failed" }
    $m = Get-Content $out | ConvertFrom-Json
    if ($m.schema -ne "rx26-episode-metrics/1") { throw "bad metrics schema: $($m.schema)" }
    Write-Host "dev setup verified - G0 smoke OK, partial credit: $($m.objective3.partial_credit)"
} else {
    Write-Host "== [3/3] skipped (-NoTest) =="
}

Write-Host ""
Write-Host "Done. Activate with:  .\.venv\Scripts\Activate.ps1"
Write-Host "Next steps: docs/SETUP_GUIDE.md section B (Run)."
