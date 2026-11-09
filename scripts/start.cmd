@echo off
setlocal DisableDelayedExpansion
rem Chill-Drone launcher. Double-click for the menu (1 fake drone, 2 real drone), or: start.cmd fake ^| real [--phone]
rem Details: apps\launcher\README.md
set "CHILL_CALLER=%CD%"
pushd "%~dp0.." || exit /b 1
chcp 65001 >nul
set "PYTHONIOENCODING=utf-8"
set "CHILL_PY=%CD%\.venv\Scripts\python.exe"
if exist "%CHILL_PY%" (
    call :check_python
    if not errorlevel 1 goto found
)
for /f "delims=" %%G in ('git rev-parse --path-format^=absolute --git-common-dir 2^>nul') do (
    if exist "%%G\..\.venv\Scripts\python.exe" (
        set "CHILL_PY=%%G\..\.venv\Scripts\python.exe"
        call :check_python
        if not errorlevel 1 goto found
    )
)
set "CHILL_PY=%CHILL_DRONE_PYTHON%"
if exist "%CHILL_PY%" (
    call :check_python
    if not errorlevel 1 goto found
)
where py.exe >nul 2>nul
if not errorlevel 1 (
    py.exe -3.12 -B -c "import sys; sys.exit(int(sys.maxsize <= 2**32))" >nul 2>nul
    if not errorlevel 1 (
        py.exe -3.12 -B -m apps.launcher %*
        goto finish
    )
)
for %%P in ("%LOCALAPPDATA%\Programs\Python\Python312\python.exe" "%ProgramFiles%\Python312\python.exe") do (
    if exist "%%~P" (
        set "CHILL_PY=%%~P"
        call :check_python
        if not errorlevel 1 goto found
    )
)
for /f "delims=" %%P in ('where python.exe 2^>nul') do (
    rem Skip the Microsoft Store app execution alias.
    if /i not "%%~dpP"=="%LOCALAPPDATA%\Microsoft\WindowsApps\" (
        set "CHILL_PY=%%P"
        call :check_python
        if not errorlevel 1 goto found
    )
)
echo [실행 전 확인] Python 3.12 ^(64비트^)를 설치한 후 scripts/start.cmd를 다시 실행해 주세요.
echo 설치 주소  https://www.python.org/downloads/windows/
pause
popd
exit /b 1
:found
"%CHILL_PY%" -B -m apps.launcher %*
:finish
set "CHILL_EXIT=%ERRORLEVEL%"
if not "%CHILL_EXIT%"=="0" pause
popd
exit /b %CHILL_EXIT%

:check_python
"%CHILL_PY%" -B -c "import sys; sys.exit(int(sys.version_info[:2] != (3, 12) or sys.maxsize <= 2**32))" >nul 2>nul
exit /b %ERRORLEVEL%
