@echo off
chcp 65001 >nul
title RTT ---- TCP bridge (for VOFA+)
set PYTHONIOENCODING=utf-8
cd /d "%~dp0"

echo ============================================================
echo   RTT  ---^>  TCP   bridge     for VOFA+
echo ------------------------------------------------------------
echo   VOFA+ settings:
echo     Data Interface = TCP Client
echo     Server IP      = 127.0.0.1
echo     Port           = 5005
echo     Data Engine    = JustFloat
echo ------------------------------------------------------------
echo   Hardware: jumper wire   PB15 (ADC)  ^<--^>  PA4 (DAC)
echo ------------------------------------------------------------
echo   ! Close this window BEFORE flashing (J-Link is locked
echo     by this program)
echo   ! Closing this window = VOFA+ receives no data
echo ============================================================
echo.

python "%~dp0rtt2tcp.py" %*

echo.
echo [bridge exited] If there is an error above, screenshot it.
echo Press any key to close...
pause >nul
