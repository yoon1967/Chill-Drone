@echo off
setlocal DisableDelayedExpansion
rem Chill-Drone launcher. Double-click for the menu (1 fake drone, 2 real drone), or: start.cmd fake ^| real [--phone]
rem Details: apps\launcher\README.md
set "CHILL_CALLER=%CD%"
pushd "%~dp0.." || exit /b 1
set "CHILL_PY=%CD%\.venv\Scripts\python.exe"
if exist "%CHILL_PY%" goto found
for /f "delims=" %%G in ('git rev-parse --path-format^=absolute --git-common-dir 2^>nul') do (
    if exist "%%G\..\.venv\Scripts\python.exe" set "CHILL_PY=%%G\..\.venv\Scripts\python.exe"
)
if exist "%CHILL_PY%" goto found
if defined CHILL_DRONE_PYTHON set "CHILL_PY=%CHILL_DRONE_PYTHON%"
if exist "%CHILL_PY%" goto found
echo [ERROR] Project Python (.venv) was not found in this checkout or the main Git worktree.
echo Create it once (1 or 2 then downloads missing packages and models with internet):
echo   py -3.12 -m venv .venv
echo Or set CHILL_DRONE_PYTHON to the python.exe to use.
pause
popd
exit /b 1
:found
for %%P in ("%CHILL_PY%") do set "CHILL_PY=%%~fP"
echo Python: "%CHILL_PY%"
"%CHILL_PY%" -B -m apps.launcher %*
set "CHILL_EXIT=%ERRORLEVEL%"
if not "%CHILL_EXIT%"=="0" pause
popd
exit /b %CHILL_EXIT%
