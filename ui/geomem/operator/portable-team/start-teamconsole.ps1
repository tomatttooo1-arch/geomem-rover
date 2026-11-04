param(
    [string]$ConfigPath = (Join-Path $PSScriptRoot 'lab-connection.json'),
    [switch]$NoBrowser,
    [switch]$ValidateOnly
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0

# This portable launcher only owns its local ssh and Node processes.
# It attaches the maintained server deployment; it never runs nano_up/car_up/car_down.
function Quote-WindowsArgument([string]$Value) {
    $result = New-Object System.Text.StringBuilder
    [void]$result.Append('"')
    $slashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq [char]92) { $slashes++; continue }
        if ($character -eq [char]34) {
            [void]$result.Append(('\' * (2 * $slashes + 1)))
            [void]$result.Append('"')
        } else {
            [void]$result.Append(('\' * $slashes))
            [void]$result.Append($character)
        }
        $slashes = 0
    }
    [void]$result.Append(('\' * (2 * $slashes)))
    [void]$result.Append('"')
    return $result.ToString()
}
function Join-WindowsArguments([string[]]$Values) {
    return (($Values | ForEach-Object { Quote-WindowsArgument $_ }) -join ' ')
}
function Quote-PosixArgument([string]$Value) {
    $escapedQuote = "'" + [char]34 + "'" + [char]34 + "'"
    return "'" + $Value.Replace("'", $escapedQuote) + "'"
}
function Quote-PowerShellArgument([string]$Value) { return "'" + $Value.Replace("'", "''") + "'" }
function Get-RequiredString($Config, [string]$Name) {
    $property = $Config.PSObject.Properties[$Name]
    if (-not $property -or $property.Value -isnot [string] -or [string]::IsNullOrWhiteSpace($property.Value)) {
        throw "Configuration field '$Name' must be a nonempty string."
    }
    return $property.Value
}
function Get-OwnedIdentity($Process, [string]$Role, [switch]$AllowExited) {
    $Process.Refresh()
    if ($Process.HasExited) {
        if ($AllowExited) { return $null }
        throw "The $Role process exited immediately. Check its log file."
    }
    try {
        return @{
            pid = $Process.Id
            startTicks = $Process.StartTime.ToUniversalTime().Ticks.ToString()
            executable = $Process.Path
            role = $Role
        }
    } catch {
        $Process.Refresh()
        if ($AllowExited -and $Process.HasExited) { return $null }
        throw
    }
}
function Stop-ExactProcess($Entry) {
    $process = Get-Process -Id ([int]$Entry.pid) -ErrorAction SilentlyContinue
    if (-not $process) { return }
    if ($process.StartTime.ToUniversalTime().Ticks.ToString() -ne [string]$Entry.startTicks -or
        $process.Path -ne [string]$Entry.executable) {
        Write-Warning "PID $($Entry.pid) no longer matches the launched process; it was left untouched."
        return
    }
    Stop-Process -InputObject $process -ErrorAction Stop
}

if ($env:OS -ne 'Windows_NT') { throw 'This launcher requires Windows PowerShell and Windows OpenSSH.' }
$ConfigPath = [System.IO.Path]::GetFullPath($ConfigPath)
if (-not (Test-Path -LiteralPath $ConfigPath -PathType Leaf)) {
    $example = Join-Path $PSScriptRoot 'lab-connection.example.json'
    if (-not (Test-Path -LiteralPath $example -PathType Leaf)) { throw "Missing example configuration: $example" }
    if (-not (Test-Path -LiteralPath (Split-Path $ConfigPath -Parent) -PathType Container)) {
        throw 'The configuration parent directory must already exist.'
    }
    Copy-Item -LiteralPath $example -Destination $ConfigPath -ErrorAction Stop
    throw "Created $ConfigPath. Edit all five connection fields, authorize your own key, verify the server fingerprint manually, then set configuration_confirmed=true. No SSH connection was attempted."
}
$config = Get-Content -LiteralPath $ConfigPath -Raw -Encoding UTF8 | ConvertFrom-Json
$confirmed = $config.PSObject.Properties['configuration_confirmed']
if (-not $confirmed -or $confirmed.Value -isnot [bool] -or -not $confirmed.Value) {
    throw 'Edit lab-connection.json first and set configuration_confirmed=true after checking all five connection fields. No SSH connection was attempted.'
}
$serverHost = Get-RequiredString $config 'host'
$serverUser = Get-RequiredString $config 'user'
$serverRepo = Get-RequiredString $config 'server_repo'
$identitySetting = Get-RequiredString $config 'identity_file'
if ($serverHost -notmatch '^(?=.{1,253}$)[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?$' -or
    @($serverHost.Split('.') | Where-Object { $_ -notmatch '^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$' }).Count) {
    throw 'host must be an IPv4 address or a DNS hostname, without whitespace or SSH options.'
}
if ($serverUser -notmatch '^[A-Za-z_][A-Za-z0-9_.-]{0,63}$') { throw 'user must be a simple Unix account name.' }
$portProperty = $config.PSObject.Properties['port']
if (-not $portProperty -or [string]$portProperty.Value -notmatch '^\d{1,5}$') { throw 'port must be an integer between 1 and 65535.' }
$serverPort = [int]$portProperty.Value
if ($serverPort -lt 1 -or $serverPort -gt 65535) { throw 'port must be between 1 and 65535.' }
if ($serverRepo -notmatch '^/(?:[A-Za-z0-9_.-]+/)*[A-Za-z0-9_.-]+/?$' -or
    @($serverRepo.Split('/') | Where-Object { $_ -eq '.' -or $_ -eq '..' }).Count) {
    throw 'server_repo must be an absolute Unix path containing only letters, digits, /, _, ., and -; dot path segments are forbidden.'
}
if ($identitySetting.IndexOfAny([char[]]@(0, 10, 13, 34)) -ge 0) { throw 'identity_file contains invalid characters.' }
if ($identitySetting -match '^~[/\\]') {
    $identity = Join-Path $env:USERPROFILE $identitySetting.Substring(2)
} elseif ([System.IO.Path]::IsPathRooted($identitySetting)) {
    $identity = $identitySetting
} else {
    $identity = Join-Path (Split-Path $ConfigPath -Parent) $identitySetting
}
$identity = [System.IO.Path]::GetFullPath($identity)
if (-not (Test-Path -LiteralPath $identity -PathType Leaf) -or $identity.EndsWith('.pub', [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "identity_file must name your existing private key, not its .pub file: $identity"
}

# Resolve all local resources and reject any occupied port before the server attach.
$packageRoot = @($PSScriptRoot, (Split-Path $PSScriptRoot -Parent)) | Where-Object {
    Test-Path -LiteralPath (Join-Path $_ 'source/ui/geomem/operator/server.mjs') -PathType Leaf
} | Select-Object -First 1
if (-not $packageRoot) { throw 'Missing source/ui/geomem/operator/server.mjs. Extract the complete handoff ZIP before launching.' }
$packageRoot = (Resolve-Path -LiteralPath $packageRoot).Path
$packagePrefix = $packageRoot.TrimEnd([char[]]@([char]92, [char]47)) + [System.IO.Path]::DirectorySeparatorChar
if ($identity.StartsWith($packagePrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw 'Keep private keys outside the extracted handoff package, for example under your user .ssh directory.'
}
$appRoot = Join-Path $packageRoot 'source/ui/geomem/operator'
$node = Join-Path $packageRoot 'runtime/node.exe'
if (-not (Test-Path -LiteralPath $node -PathType Leaf)) { throw "Missing bundled Node runtime: $node" }
$nodeVersion = (& $node --version | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or $nodeVersion -notmatch '^v(\d+)\.' -or [int]$Matches[1] -lt 20) {
    throw 'The bundled Node runtime must run successfully and be version 20 or newer.'
}
$ssh = (Get-Command ssh.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
Get-Command Get-NetTCPConnection -ErrorAction Stop | Out-Null
$stateDir = Join-Path $PSScriptRoot '.team-console'
$stateFile = Join-Path $stateDir 'current.json'
if (Test-Path -LiteralPath $stateFile) {
    throw 'A previous local launch record exists. Run stop-teamconsole.ps1 first; it verifies process identities before stopping only its own local services.'
}
$ports = @(8788, 8790, 8792)
$occupied = @([System.Net.NetworkInformation.IPGlobalProperties]::GetIPGlobalProperties().GetActiveTcpListeners() |
    Where-Object { $_.Port -in $ports })
if ($occupied.Count) {
    throw "Local console port(s) already occupied: $((@($occupied.Port | Sort-Object -Unique)) -join ', '). Close the existing launcher yourself. This launcher will not adopt or stop another service."
}
$server = "$serverUser@$serverHost"
$baseArgs = @('-F', 'NUL', '-i', $identity, '-p', $serverPort.ToString(),
    '-o', 'IdentitiesOnly=yes', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
    '-o', 'ConnectTimeout=8', '-o', 'ServerAliveInterval=15', '-o', 'ServerAliveCountMax=3')
$manualArgs = @('-F', 'NUL', '-i', $identity, '-p', $serverPort.ToString(),
    '-o', 'IdentitiesOnly=yes', '-o', 'StrictHostKeyChecking=ask', $server, 'exit')
Write-Host 'Before first use, manually verify the server fingerprint with the owner and ensure this key is authorized.'
Write-Host 'If OpenSSH rejects private-key permissions, fix the key ACL for your Windows account; do not weaken host-key checks.'
Write-Host ('Manual SSH command: & ' + (Quote-PowerShellArgument $ssh) + ' ' + (($manualArgs | ForEach-Object { Quote-PowerShellArgument $_ }) -join ' '))
if ($ValidateOnly) {
    Write-Host "Local configuration and resources are valid ($nodeVersion). No network connection or service launch was attempted."
    exit 0
}

$url = 'http://127.0.0.1:8788/#remote'
New-Item -ItemType Directory -Force -Path $stateDir | Out-Null
$logDir = Join-Path $stateDir ((Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $logDir | Out-Null
$owned = @()
$attachIdentity = $null
try {
    Write-Host 'Attaching the existing READY car stack through the maintained server deployment...'
    $remoteCommand = 'cd -- ' + (Quote-PosixArgument $serverRepo) + ' && bash ' + (Quote-PosixArgument 'ui/geomem/tools/competition/connect_existing.sh')
    # Start-Process -PassThru can leave ExitCode null after redirected output on Windows PowerShell.
    # Start the short-lived attach command directly so its exit code is reliable.
    $attachInfo = New-Object System.Diagnostics.ProcessStartInfo
    $attachInfo.FileName = $ssh
    $attachInfo.Arguments = Join-WindowsArguments ($baseArgs + @($server, $remoteCommand))
    $attachInfo.UseShellExecute = $false
    $attachInfo.CreateNoWindow = $true
    $attachInfo.RedirectStandardOutput = $true
    $attachInfo.RedirectStandardError = $true
    $attach = [System.Diagnostics.Process]::Start($attachInfo)
    $attachIdentity = Get-OwnedIdentity $attach 'server-attach' -AllowExited
    $attachOut = $attach.StandardOutput.ReadToEndAsync()
    $attachErr = $attach.StandardError.ReadToEndAsync()
    if (-not $attach.WaitForExit(45000)) { throw "Server attachment timed out. Logs: $logDir" }
    [System.IO.File]::WriteAllText((Join-Path $logDir 'attach.out.log'), $attachOut.Result)
    [System.IO.File]::WriteAllText((Join-Path $logDir 'attach.err.log'), $attachErr.Result)
    $attachExitCode = $attach.ExitCode
    if ($attachExitCode -ne 0) {
        Get-Content -LiteralPath (Join-Path $logDir 'attach.err.log') -ErrorAction SilentlyContinue | Write-Host
        Get-Content -LiteralPath (Join-Path $logDir 'attach.out.log') -ErrorAction SilentlyContinue | Write-Host
        throw "Server attachment failed (exit $attachExitCode). Check network/VPN, authorized key, known host, Nano connection, and the existing READY stack. This launcher did not start the car stack. Logs: $logDir"
    }
    $tunnelArgs = $baseArgs + @('-o', 'ExitOnForwardFailure=yes', '-N',
        '-L', '127.0.0.1:8790:127.0.0.1:8790', '-L', '127.0.0.1:8792:127.0.0.1:28792', $server)
    $tunnel = Start-Process -FilePath $ssh -ArgumentList (Join-WindowsArguments $tunnelArgs) -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir 'ssh.out.log') -RedirectStandardError (Join-Path $logDir 'ssh.err.log')
    $owned += Get-OwnedIdentity $tunnel 'ssh-tunnel'
    $environmentBefore = @{}
    try {
        foreach ($name in @('PORT', 'HOST', 'GEOMEM_BRIDGE_PORT', 'GEOMEM_MAPPING_PORT')) {
            $environmentBefore[$name] = [System.Environment]::GetEnvironmentVariable($name, 'Process')
        }
        $env:PORT = '8788'; $env:HOST = '127.0.0.1'; $env:GEOMEM_BRIDGE_PORT = '8790'; $env:GEOMEM_MAPPING_PORT = '8792'
        $web = Start-Process -FilePath $node -ArgumentList (Join-WindowsArguments @((Join-Path $appRoot 'server.mjs'))) -WorkingDirectory $appRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logDir 'web.out.log') -RedirectStandardError (Join-Path $logDir 'web.err.log')
        $owned += Get-OwnedIdentity $web 'web'
    } finally {
        foreach ($name in $environmentBefore.Keys) { [System.Environment]::SetEnvironmentVariable($name, $environmentBefore[$name], 'Process') }
    }
    $deadline = (Get-Date).AddSeconds(12)
    $ready = @()
    do {
        foreach ($entry in $owned) {
            $process = Get-Process -Id $entry.pid -ErrorAction SilentlyContinue
            if (-not $process -or $process.StartTime.ToUniversalTime().Ticks.ToString() -ne $entry.startTicks) {
                throw "A local console service exited. Logs: $logDir"
            }
        }
        $listeners = @(Get-NetTCPConnection -State Listen -ErrorAction SilentlyContinue | Where-Object { $_.LocalPort -in $ports })
        $unexpected = @($listeners | Where-Object {
            ($_.LocalPort -eq 8788 -and $_.OwningProcess -ne $web.Id) -or
            ($_.LocalPort -in @(8790, 8792) -and $_.OwningProcess -ne $tunnel.Id)
        })
        if ($unexpected.Count) { throw "Another process acquired a console port during launch. It was left untouched. Logs: $logDir" }
        $ready = @($listeners | Where-Object { $_.LocalAddress -eq '127.0.0.1' -and $_.OwningProcess -in @($web.Id, $tunnel.Id) })
        if (@($ready | ForEach-Object { $_.LocalPort } | Sort-Object -Unique).Count -eq 3) { break }
        Start-Sleep -Milliseconds 200
    } while ((Get-Date) -lt $deadline)
    if (@($ready | ForEach-Object { $_.LocalPort } | Sort-Object -Unique).Count -ne 3) { throw "Local console ports did not become ready. Logs: $logDir" }
    @{
        schema = 1
        package_root = $packageRoot
        launcher_root = $PSScriptRoot
        mode = 'team-server-attach'
        url = $url
        processes = @($owned)
        logs = $logDir
        started_at = (Get-Date).ToString('o')
    } | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $stateFile -Encoding UTF8
    Write-Host "READY: $url"
    Write-Host 'Local gateway is ready. Check the page for the live car status before enabling manual driving.'
    Write-Host 'stop-teamconsole.ps1 only closes these local services; it does not stop the car stack.'
    if (-not $NoBrowser) { Start-Process $url }
} catch {
    foreach ($entry in $owned) { Stop-ExactProcess $entry }
    if ($attachIdentity) { Stop-ExactProcess $attachIdentity }
    throw
}
