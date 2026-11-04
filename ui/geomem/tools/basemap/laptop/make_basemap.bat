@echo off
rem FPGA basemap portable launcher (Windows, no Docker / no install).
rem   make_basemap.bat <map.db> <out_dir> [more make_basemap.py args, e.g. --force --threads 12]
rem   -> <out_dir>\basemap-<sha12>\{basemap.json,cells.bin}
rem Repo root: env FPGA_ROOT, default = parent folder of this package (i.e. unzip into <ROOT>\fpga-basemap-win64\).
setlocal EnableExtensions
set "PKG=%~dp0"
if not defined FPGA_ROOT set "FPGA_ROOT=%PKG%.."
set "MB=%FPGA_ROOT%\ui\geomem\tools\basemap\make_basemap.py"
if "%~2"=="" (
  echo usage: make_basemap.bat ^<map.db^> ^<out_dir^> [extra args]
  exit /b 2
)
if not exist "%MB%" (
  echo [fpga-basemap] not found: %MB%
  echo [fpga-basemap] set FPGA_ROOT=^<repo root^> or unzip this package inside the repo root
  exit /b 3
)
if not exist "%~1" (
  echo [fpga-basemap] db not found: %~1
  exit /b 4
)
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
set "DB=%~f1"
set "OUT=%~f2"
set "REST="
:collect
if "%~3"=="" goto run
set REST=%REST% "%~3"
shift /3
goto collect
:run
echo [fpga-basemap] start %DATE% %TIME%
"%PKG%python\python.exe" "%PKG%fpga_run.py" "%MB%" --db "%DB%" --out-dir "%OUT%" %REST%
set "RC=%ERRORLEVEL%"
echo [fpga-basemap] end   %DATE% %TIME%  rc=%RC%
if "%RC%"=="0" echo [fpga-basemap] result: "%OUT%\basemap-*" (see "basemap ..." line above)
exit /b %RC%
