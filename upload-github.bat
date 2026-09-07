@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0upload-github.ps1" %*
set "upload_result=%errorlevel%"
echo.
pause
exit /b %upload_result%
