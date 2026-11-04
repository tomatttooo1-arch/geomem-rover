param(
    [string]$ConfigPath = (Join-Path $PSScriptRoot 'lab-connection.json'),
    [switch]$NoBrowser
)
# 一键打开比赛控制台（直连 Nano，10-21）。双击 start-directconsole.cmd 即可：
#   1. 找 Nano：先用配置里的地址；连不上就经服务器查它当前的校园网地址并记下来
#   2. 第一次用时把你的公钥装到 Nano（只输一次 Nano 密码），以后免密
#   3. 车栈没起就问你要不要起（会让底盘上电，进入可遥控状态）
#   4. 建一条 ssh 隧道，打开浏览器。关掉这个窗口＝断开（车栈继续运行）
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }
$OutputEncoding = [System.Text.Encoding]::UTF8

$NanoUser = 'robot'
$Remote = '/home/robot/fpga/dev1/console-current/ui/geomem/tools/competition/console_remote.sh'
$LocalPort = 8788
$Url = "http://127.0.0.1:$LocalPort/#remote"

$config = $null
if (Test-Path -LiteralPath $ConfigPath) { $config = Get-Content -LiteralPath $ConfigPath -Raw -Encoding UTF8 | ConvertFrom-Json }
function Get-Setting([string]$Name, [string]$Default) {
    if ($config -and $config.PSObject.Properties[$Name] -and [string]$config.$Name) { return [string]$config.$Name }
    return $Default
}
$identity = Get-Setting 'identity_file' '~/.ssh/id_ed25519'
if ($identity -match '^~[/\\]') { $identity = Join-Path $env:USERPROFILE $identity.Substring(2) }
if (-not (Test-Path -LiteralPath $identity)) { throw "找不到你的 SSH 私钥：$identity（在 lab-connection.json 的 identity_file 里改）" }
$serverHost = Get-Setting 'host' '198.18.200.1'
$serverPort = Get-Setting 'port' '2222'
$serverUser = Get-Setting 'user' 'robot'
$nanoHost = Get-Setting 'nano_host' ''
$ssh = (Get-Command ssh.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
$common = @('-i', $identity, '-o', 'IdentitiesOnly=yes', '-o', 'ConnectTimeout=6', '-o', 'StrictHostKeyChecking=accept-new')

function Test-Tcp([string]$TargetHost, [int]$Port, [int]$TimeoutMs = 2500) {
    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $wait = $client.BeginConnect($TargetHost, $Port, $null, $null)
        return ($wait.AsyncWaitHandle.WaitOne($TimeoutMs) -and $client.Connected)
    } catch { return $false } finally { $client.Close() }
}
# Windows PowerShell 5.1 turns a native program's stderr into error records; with 'Stop' that would abort.
function Invoke-Native([string[]]$NativeArgs) {
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & $ssh @NativeArgs } finally { $ErrorActionPreference = $saved }
}
function Invoke-Nano([string[]]$Command, [switch]$Batch) {
    $sshArgs = $common
    if ($Batch) { $sshArgs += @('-o', 'BatchMode=yes') }
    Invoke-Native ($sshArgs + @("$NanoUser@$nanoHost") + $Command)
}

# 0. 已经连着：直接开浏览器
if (Test-Tcp '127.0.0.1' $LocalPort 500) {
    try {
        $page = Invoke-WebRequest -UseBasicParsing -TimeoutSec 3 -Uri "http://127.0.0.1:$LocalPort/index.html"
        if ($page.Content -match 'GeoMem') {
            Write-Host '控制台已经连着，直接打开浏览器。'
            if (-not $NoBrowser) { Start-Process $Url }
            exit 0
        }
    } catch { }
    throw "本机端口 $LocalPort 被别的程序占用（旧的服务器控制台？先运行 stop_local_console.cmd 或关掉那个窗口）。"
}

# 1. 找 Nano
if (-not $nanoHost -or -not (Test-Tcp $nanoHost 22)) {
    Write-Host '连不上配置里的 Nano 地址，正在通过服务器查询它当前的地址……'
    $found = Invoke-Native ($common + @('-o', 'BatchMode=yes', '-p', $serverPort, "$serverUser@$serverHost", 'ssh -o BatchMode=yes -o ConnectTimeout=8 jetson hostname -I')) 2>$null
    # 车上小米路由器（198.18.31.x）优先，其次校园网 10.x；10.42.x 是 KV260 直连口，不算
    $candidates = @(([string]$found) -split '\s+' | Where-Object { $_ -match '^192\.168\.31\.' }) +
                  @(([string]$found) -split '\s+' | Where-Object { $_ -match '^10\.' -and $_ -notmatch '^10\.42\.' })
    $ip = $candidates | Where-Object { Test-Tcp $_ 22 } | Select-Object -First 1
    if (-not $ip) {
        throw '连不上 Nano。请确认：Nano 已开机；这台电脑和 Nano 在同一个网里（都连车上小米路由器 Xiaomi_1796_5G，或都在同一个局域网）。也可以把 Nano 地址直接写进 lab-connection.json 的 nano_host。'
    }
    $nanoHost = $ip
    Write-Host "Nano 当前地址：$nanoHost"
    if ($config) {
        $config | Add-Member -NotePropertyName 'nano_host' -NotePropertyValue $nanoHost -Force
        $config | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $ConfigPath -Encoding UTF8
    }
}

# 2. 免密：第一次装公钥
Invoke-Nano @('true') -Batch 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host '第一次连这台 Nano：把你的公钥装上去，接下来只需输入一次 Nano 的密码。'
    $pubPath = "$identity.pub"
    $pub = if (Test-Path -LiteralPath $pubPath) { (Get-Content -LiteralPath $pubPath -Raw).Trim() } else { (& ssh-keygen.exe -y -f $identity).Trim() }
    if ($pub -notmatch '^ssh-') { throw '读不出公钥。' }
    $pub | & $ssh @common "$NanoUser@$nanoHost" 'umask 077; mkdir -p ~/.ssh; sed ''s/^\xEF\xBB\xBF//; s/\r$//'' >> ~/.ssh/authorized_keys'
    Invoke-Nano @('true') -Batch 2>$null
    if ($LASTEXITCODE -ne 0) { throw '公钥安装后仍无法免密登录 Nano（密码输错？）。' }
    Write-Host '已装好，以后不再问密码。'
}

# 3. 车栈
$status = (Invoke-Nano @('bash', $Remote, 'status') -Batch) -join "`n"
if ($LASTEXITCODE -ne 0 -or $status -notmatch 'STACK') { throw "读不到 Nano 上的车栈状态：$status" }
if ($status -match 'STACK DOWN') {
    $answer = Read-Host '车栈没有运行。现在启动吗？启动后底盘上电、进入可遥控状态，请确认车周围安全 [Y/N]'
    if ($answer -notmatch '^[Yy]') { Write-Host '没有启动车栈，退出。'; exit 1 }
    Write-Host '正在启动车栈（约 1–2 分钟）……'
    Invoke-Nano @('bash', $Remote, 'up') -Batch
    if ($LASTEXITCODE -ne 0) { throw '车栈没有启动成功，看上面的输出。' }
} elseif ($status -match 'GATEWAY DOWN') {
    Invoke-Nano @('bash', $Remote, 'up') -Batch
    if ($LASTEXITCODE -ne 0) { throw '控制台网关没有起来，看上面的输出。' }
}

# 4. 隧道 + 浏览器
$tunnelArgs = $common + @('-N', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes', '-o', 'ServerAliveInterval=5',
    '-o', 'ServerAliveCountMax=3', '-L', "127.0.0.1:${LocalPort}:127.0.0.1:8790", "$NanoUser@$nanoHost")
$quoted = $tunnelArgs | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }   # Start-Process joins with spaces
$tunnel = Start-Process -FilePath $ssh -ArgumentList $quoted -NoNewWindow -PassThru
$null = $tunnel.Handle
$up = $false
for ($i = 0; $i -lt 50 -and -not $tunnel.HasExited; $i++) {
    if (Test-Tcp '127.0.0.1' $LocalPort 300) { $up = $true; break }
    Start-Sleep -Milliseconds 200
}
if (-not $up) {
    if (-not $tunnel.HasExited) { Stop-Process -Id $tunnel.Id -Force }
    throw '隧道没有建起来。'
}
Write-Host "已连接 Nano（$nanoHost）。控制台：$Url"
if (-not $NoBrowser) { Start-Process $Url }
Write-Host '开车期间保持这个窗口打开；关掉窗口＝断开控制台（车栈继续运行，关车栈用 stop-carstack.cmd）。'
$tunnel.WaitForExit()
Write-Host '隧道已断开。重新双击 start-directconsole.cmd 即可再连上。'
