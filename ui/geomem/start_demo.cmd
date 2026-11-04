@echo off
setlocal
chcp 65001 >nul
title GeoMem M8
cd /d "%~dp0"
if not exist "%~dp0runtime\node.exe" (
  echo Runtime missing. Please extract the ENTIRE ZIP first.
  pause
  exit /b 1
)
"%~dp0runtime\node.exe" "%~dp0start.mjs" %*
if errorlevel 1 pause
