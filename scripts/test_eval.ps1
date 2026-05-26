# Closed-loop model eval one-liner. Append future flags to $evalArgs below.
# Run from repo root:
#
#   pwsh -File scripts/test_eval.ps1
#   # or in PowerShell:
#   .\scripts\test_eval.ps1
#
# Pass extra flags through, e.g.:
#   .\scripts\test_eval.ps1 --max-steps 500 --turn-deg 4

$ErrorActionPreference = "Stop"

# Resolve repo root (parent of this scripts/ folder) so script works from anywhere.
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$evalArgs = @(
    "--model",                 "models/turn_classifier.pkl",
    "--map",                   "data/maps/outline_trace4_w.json",
    "--curve",                 "catmull_rom",
    "--curve-samples",         "48",
    "--run-name",              "eval_spawn",
    "--output-dir",            "data/evals",
    "--max-steps",             "300",
    "--forward-step-cm",       "12.5",      # 0.5 step-style forward move
    "--turn-deg",              "5.0",
    "--finish-threshold-cm",   "30.0",
    "--centerline-samples",    "2000",
    "--spawn-capture-camera",
    "--force-camera-follow",
    "--camera-x-offset-cm",    "0.0",
    "--camera-z-offset-cm",    "14.846071",
    "--camera-pitch-deg",      "-7.0",
    "--settle",                "0.08",
    "--debug"
    # Append additional defaults here as needed.
)

if ($args.Count -gt 0) {
    $evalArgs += $args
}

Write-Host "Running: python eval.py $($evalArgs -join ' ')"
& python eval.py @evalArgs
exit $LASTEXITCODE

