param([string]$ConfigPath = (Join-Path $PSScriptRoot 'lab-connection.json'))
# 关掉 Nano 上的车栈和控制台网关（收车用，10-21）。只在车已停稳时用。
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version 2.0
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }
$config = $null
if (Test-Path -LiteralPath $ConfigPath) { $config = Get-Content -LiteralPath $ConfigPath -Raw -Encoding UTF8 | ConvertFrom-Json }
$identity = if ($config -and $config.PSObject.Properties['identity_file']) { [string]$config.identity_file } else { '~/.ssh/id_ed25519' }
if ($identity -match '^~[/\\]') { $identity = Join-Path $env:USERPROFILE $identity.Substring(2) }
$nanoHost = if ($config -and $config.PSObject.Properties['nano_host']) { [string]$config.nano_host } else { '' }
if (-not $nanoHost) { throw '先用 start-directconsole.cmd 连一次，让它记下 Nano 地址。' }
$answer = Read-Host "关闭 Nano（$nanoHost）上的车栈？车会断电停止，建图未保存的部分会丢 [Y/N]"
if ($answer -notmatch '^[Yy]') { exit 1 }
$ssh = (Get-Command ssh.exe -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
& $ssh -i $identity -o IdentitiesOnly=yes -o BatchMode=yes -o ConnectTimeout=6 "robot@$nanoHost" bash /home/robot/fpga/dev1/console-current/ui/geomem/tools/competition/console_remote.sh down
if ($LASTEXITCODE -ne 0) { throw '关闭失败，看上面的输出。' }
Write-Host '车栈已关闭。'
