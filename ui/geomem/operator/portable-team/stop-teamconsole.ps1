param()
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
$stateFile = Join-Path $PSScriptRoot '.team-console/current.json'
if (-not (Test-Path -LiteralPath $stateFile -PathType Leaf)) {
    Write-Host 'No local launch record exists. No process was stopped.'
    exit 0
}
$state = Get-Content -LiteralPath $stateFile -Raw -Encoding UTF8 | ConvertFrom-Json
if ($state.schema -ne 1 -or $state.mode -ne 'team-server-attach' -or
    [string]$state.launcher_root -ne $PSScriptRoot -or @($state.processes).Count -ne 2) {
    throw 'Unrecognized launch record. No process was stopped. Inspect .team-console/current.json manually.'
}
$roles = @($state.processes.role | Sort-Object)
if (($roles -join ',') -ne 'ssh-tunnel,web') { throw 'Invalid process roles. No process was stopped.' }
$pending = @()
# Validate every existing PID before stopping any process.
foreach ($entry in $state.processes) {
    if ([string]$entry.pid -notmatch '^[1-9][0-9]*$' -or [string]$entry.startTicks -notmatch '^[0-9]+$' -or
        [string]::IsNullOrWhiteSpace([string]$entry.executable)) {
        throw 'Invalid process identity in the launch record. No process was stopped.'
    }
    $process = Get-Process -Id ([int]$entry.pid) -ErrorAction SilentlyContinue
    if (-not $process) { continue }
    if ($process.StartTime.ToUniversalTime().Ticks.ToString() -ne [string]$entry.startTicks -or
        $process.Path -ne [string]$entry.executable) {
        throw "PID $($entry.pid) no longer matches the launched process. No process was stopped; do not kill it based on this stale record."
    }
    $pending += @{ process = $process; entry = $entry }
}
foreach ($item in $pending) {
    # Recheck immediately before stopping, including start time to prevent PID reuse.
    $process = Get-Process -Id ([int]$item.entry.pid) -ErrorAction SilentlyContinue
    if (-not $process) { continue }
    if ($process.StartTime.ToUniversalTime().Ticks.ToString() -ne [string]$item.entry.startTicks -or
        $process.Path -ne [string]$item.entry.executable) {
        throw "PID $($item.entry.pid) changed while stopping. Remaining processes were left untouched."
    }
    Stop-Process -InputObject $process -ErrorAction Stop
}
Remove-Item -LiteralPath $stateFile -ErrorAction Stop
Write-Host 'Closed only this launcher''s local SSH tunnel and web gateway. The car stack, camera, map session, server bridge, and other launchers were left running.'
Write-Host 'This is not a vehicle stop or emergency stop. Release driving controls and disable manual driving in the page before closing it.'
