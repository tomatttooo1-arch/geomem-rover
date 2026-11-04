param([switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
$appRoot = Split-Path $PSScriptRoot -Parent
$session = @(
    (Join-Path $appRoot '../tools/competition/session.py'),
    (Join-Path $appRoot '../competition/ui/geomem/tools/competition/session.py')
) | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $session) { throw 'Missing competition source package.' }
$stateDir = Join-Path (Split-Path (Resolve-Path -LiteralPath $session).Path -Parent) 'session-data/lab-console'
New-Item -ItemType Directory -Force -Path $stateDir | Out-Null
$stateFile = Join-Path $stateDir 'current.json'
$url = 'http://127.0.0.1:8788/#remote'
$ssh = (Get-Command ssh -ErrorAction Stop).Source
$identity = Join-Path $env:USERPROFILE '.ssh/geomem-rover-20261018'
$serverArgs = @('-F', 'NUL', '-i', $identity, '-p', '2222', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15')
$server = 'robot@198.18.200.1'
Write-Host 'Connecting through the laboratory server to the existing car stack...'
& $ssh @serverArgs $server 'cd ${FPGA_SRC} && bash ui/geomem/tools/competition/connect_existing.sh'
if ($LASTEXITCODE -ne 0) { throw 'Car attachment failed. See the message above; no car stack was started by this launcher.' }

$ports = @(8788, 8790, 8792)
$listeners = @(Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object { $_.LocalAddress -eq '127.0.0.1' -and $_.LocalPort -in $ports })
if ($listeners.Count) {
    if (-not (Test-Path -LiteralPath $stateFile)) { throw 'A local console port is occupied by another launcher. Close that launcher first.' }
    $saved = Get-Content -LiteralPath $stateFile -Raw -Encoding UTF8 | ConvertFrom-Json
    foreach ($entry in $saved.processes) {
        $process = Get-Process -Id $entry.pid -ErrorAction Stop
        if ($process.StartTime.ToUniversalTime().Ticks.ToString() -ne $entry.startTicks) { throw 'Saved console process identity changed; inspect occupied ports.' }
    }
    foreach ($port in $ports) {
        if (-not ($listeners | Where-Object { $_.LocalPort -eq $port -and $_.OwningProcess -in $saved.processes.pid })) { throw 'Console services are incomplete or another process occupies a port.' }
    }
    Write-Host "Console already running: $url"
    if (-not $NoBrowser) { Start-Process $url }
    exit 0
}

$node = @(
    (Join-Path $appRoot '../../../../runtime/node.exe'),
    (Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe'),
    (Get-Command node -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source)
) | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -First 1
if (-not $node) { throw 'Node.js 20+ is required.' }
$logDir = Join-Path $stateDir (Get-Date -Format 'yyyyMMdd-HHmmss')
New-Item -ItemType Directory -Path $logDir | Out-Null
$owned = @()
try {
    $tunnelArgs = @('-F', 'NUL', '-i', ('"' + $identity + '"'), '-p', '2222', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=15', '-N', '-L', '127.0.0.1:8790:127.0.0.1:8790', '-L', '127.0.0.1:8792:127.0.0.1:28792', $server)
    $owned += Start-Process -FilePath $ssh -ArgumentList $tunnelArgs -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir 'ssh.out.log') -RedirectStandardError (Join-Path $logDir 'ssh.err.log')
    $env:PORT = '8788'
    $env:GEOMEM_BRIDGE_PORT = '8790'
    $env:GEOMEM_MAPPING_PORT = '8792'
    $owned += Start-Process -FilePath $node -ArgumentList ('"' + (Join-Path $appRoot 'server.mjs') + '"') -WorkingDirectory $appRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir 'web.out.log') -RedirectStandardError (Join-Path $logDir 'web.err.log')
    $deadline = (Get-Date).AddSeconds(12)
    do {
        if ($owned | Where-Object { $_.HasExited }) { throw "A console service exited. Logs: $logDir" }
        $ready = @(Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object { $_.LocalAddress -eq '127.0.0.1' -and $_.LocalPort -in $ports -and $_.OwningProcess -in $owned.Id })
        if (@($ready.LocalPort | Sort-Object -Unique).Count -eq 3) { break }
        Start-Sleep -Milliseconds 200
    } while ((Get-Date) -lt $deadline)
    if (@($ready.LocalPort | Sort-Object -Unique).Count -ne 3) { throw "Console ports did not become ready. Logs: $logDir" }
    @{ mode = 'laboratory-server'; url = $url; processes = @($owned | ForEach-Object { @{ pid = $_.Id; startTicks = $_.StartTime.ToUniversalTime().Ticks.ToString() } }); logs = $logDir } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $stateFile -Encoding UTF8
    Write-Host "READY: $url"
    if (-not $NoBrowser) { Start-Process $url }
} catch {
    foreach ($process in $owned) { if (-not $process.HasExited) { Stop-Process -Id $process.Id -ErrorAction SilentlyContinue } }
    throw
}
