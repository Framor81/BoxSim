# Test-collection one-liner for outline trace 3 (world map). Append future flags to the @args array below
# instead of typing them out each time. Run from repo root:
#
#   pwsh -File scripts/test_collect3.ps1
#   # or in PowerShell:
#   .\scripts\test_collect3.ps1
#
# Pass extra flags through, e.g.:
#   .\scripts\test_collect3.ps1 --turn-threshold 10 --no-capture

$ErrorActionPreference = "Stop"

# Resolve repo root (parent of this scripts/ folder) so the script works no
# matter where it is invoked from.
$repoRoot = Split-Path -Parent $PSScriptRoot
Set-Location $repoRoot

$collectArgs = @(
    "--run-name",            "track3",
    "--map",                 "data/maps/outline_trace3_w.json",
    "--step",                "2",
    "--lookahead",           "2",
    "--spawn-capture-camera",
    "--force-camera-follow",
    "--follow-spline-forward",           # forward steps follow spline arc (stay on tape)
    "--finish-threshold-cm", "12",       # must be this close to path end (cm); tighter than 30
    "--min-finish-progress-ratio", "0.97",
    "--camera-x-offset-cm",  "1",
    "--camera-z-offset-cm",  "10",
    "--camera-pitch-deg",    "-17.5",
    "--camera-fov-deg",      "100.0",       # default ~90; wider lens
    "--turn-step-deg",       "2.5",         # smaller turning increments
    "--center-tol-cm",       "6.0",         # forward-preferred centered band
    "--heading-tol-deg",     "4.0",
    "--max-consecutive-turns","2",          # can turn twice before forced forward
    "--max-consecutive-forwards", "6",      # split-yaw: inject turn after N forwards if still misaligned
    "--forward-step-cm",     "1.0",         # 0.5 * --step (step=2)
    "--post-turn-forward-scale","0.5",      # stricter centered gate after turns
    "--align-threshold",     "4.0",         # legacy/no-op compatibility knob
    "--turn-threshold",      "15.0",        # manifest semantic_turn on rotation rows
    "--randomization",                      # slight per-run noise for diversity
    "--num-runs",            "3",           # writes track3_r001, _r002, _r003
    "--debug"
    # Append additional flags here as the workflow evolves, e.g.:
    # "--waypoint-radius-cm", "30",
)

if ($args.Count -gt 0) {
    $collectArgs += $args
}

Write-Host "Running: python data_collection/collect.py $($collectArgs -join ' ')"
& python data_collection/collect.py @collectArgs
exit $LASTEXITCODE
