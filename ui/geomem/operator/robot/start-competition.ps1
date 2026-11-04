param([string]$NanoHost)
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$appRoot = Split-Path $PSScriptRoot -Parent
$candidates = @(
  (Join-Path $appRoot '../tools/competition/session.py'),
  (Join-Path $appRoot '../competition/ui/geomem/tools/competition/session.py')
)
$session = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $session) { throw '完整交付包中的 session.py 缺失；请保留 source 与启动器。' }
$session = (Resolve-Path -LiteralPath $session).Path
$configFile = Join-Path (Split-Path $session -Parent) 'session.json'
if (-not (Test-Path -LiteralPath $configFile)) {
  Copy-Item -LiteralPath (Join-Path (Split-Path $session -Parent) 'session.example.json') -Destination $configFile
}
$config = Get-Content -LiteralPath $configFile -Raw -Encoding UTF8 | ConvertFrom-Json
if (-not $NanoHost -and $config.nano_ip -eq '198.18.1.100') {
  $NanoHost = Read-Host '请输入同一局域网 Nano 的 IP 地址（不要填写实验室服务器）'
}
if ($NanoHost) {
  if ($NanoHost -notmatch '^[a-zA-Z0-9.-]+$') { throw 'Nano 地址格式无效。' }
  $config.nano_ip = $NanoHost
  $config.nano_ssh = 'robot@' + $NanoHost
}
$config | Add-Member -NotePropertyName operator_root -NotePropertyValue $appRoot -Force
$nodeCommand = Get-Command node -ErrorAction SilentlyContinue
$node = if ($nodeCommand) { $nodeCommand.Source } else { $null }
if (-not $node) {
  $node = @(
    (Join-Path $appRoot '../../../../runtime/node.exe'),
    (Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe')
  ) | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $node) { throw '找不到 Node.js；请保留交付包 runtime/node.exe，或安装 Node.js 20 及以上版本。' }
$node = (Resolve-Path -LiteralPath $node).Path
$config | Add-Member -NotePropertyName node -NotePropertyValue $node -Force
$config | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $configFile -Encoding UTF8
$python = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
if (-not (Test-Path -LiteralPath $python)) { $python = (Get-Command python -ErrorAction Stop).Source }
Write-Host '电脑直接连接 Nano，再由 Nano 连接 KV260。请保持本窗口打开。'
& $python $session start --config $configFile --execute
if ($LASTEXITCODE -ne 0) { throw "直连启动没有完成（退出码 $LASTEXITCODE）。按上方原因检查；不会自动回退到实验室服务器。" }
