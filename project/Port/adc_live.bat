@echo off
REM ===========================================================================
REM  adc_live.bat -- live monitor for the ADC input on PB15 (ADC2_IN15)
REM
REM  Shows min / max / peak-to-peak voltage on PB15 in real time.
REM    pk near 4000 -> a real signal is present
REM    pk near 0    -> the pin is a flat line (nothing connected / wrong pin)
REM
REM  Close the RTT bridge first (J-Link is exclusive).
REM  Press Ctrl+C to stop.
REM ===========================================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0adc_live.ps1"
pause
