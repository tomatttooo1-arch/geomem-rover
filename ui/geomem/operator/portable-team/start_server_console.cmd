@echo off
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-TeamConsole.ps1"
if errorlevel 1 echo Launcher failed. Read the message above and the handoff document.
pause
