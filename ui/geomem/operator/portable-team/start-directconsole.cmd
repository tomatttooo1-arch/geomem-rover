@echo off
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0Start-DirectConsole.ps1"
if errorlevel 1 echo Direct console failed. Read the message above.
pause
