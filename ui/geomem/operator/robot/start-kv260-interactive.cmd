@echo off
title KV260 service startup - enter sudo password here
echo Starting KV260 service only. No drive or navigation command is sent.
ssh -t -o ClearAllForwardings=yes srtp-server "ssh -t kv260 'sudo bash /home/ubuntu/fpga/dev3-competition-20261019/repo/ui/geomem/tools/competition/kv_start.sh /home/ubuntu/fpga/dev3-competition-20261019/repo /home/ubuntu/fpga/dev3-competition-20261019/run-01'"
echo.
echo Exit code: %ERRORLEVEL%
pause
