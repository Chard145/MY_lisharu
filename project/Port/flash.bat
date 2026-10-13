@echo off
REM ===========================================================================
REM  flash.bat - double-click entry for the verified flash flow
REM ---------------------------------------------------------------------------
REM  Runs Port\flash_verify.ps1 which builds, flashes, then READS THE CHIP BACK
REM  and compares it byte-for-byte with the image. Prints PASS or FAIL.
REM
REM  ASCII only: cmd.exe parses .bat as GBK and non-ASCII breaks it.
REM ===========================================================================
setlocal
set PS1=%~dp0flash_verify.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File "%PS1%" %*
echo.
echo ---------------------------------------------------------------
echo  Exit code: %ERRORLEVEL%   (0 = PASS, 1 = FAIL)
echo ---------------------------------------------------------------
pause
