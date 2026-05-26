# Merge inch outline_traceN.json + outline_traceN_w.json -> outline_traceN_w.json (trace4-style).
# Usage (repo root): .\scripts\merge_world_map.ps1 1
param(
    [Parameter(Position = 0)]
    [string] $Trace = "1"
)
Set-Location (Split-Path $PSScriptRoot -Parent)
python scripts/normalize_map_json.py $Trace
