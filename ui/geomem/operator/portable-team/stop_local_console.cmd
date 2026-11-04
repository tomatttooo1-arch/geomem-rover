@echo off
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Stop-TeamConsole.ps1"
if errorlevel 1 echo Local shutdown refused. Read the message above; do not kill unrelated processes.
pause
