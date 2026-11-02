@echo off
setlocal DisableDelayedExpansion
pushd "%~dp0.." || exit /b 1
if not exist ".venv\Scripts\python.exe" (
    echo [ERROR] Project Python is missing: .venv\Scripts\python.exe
    echo Create it with: py -3 -m venv .venv
    pause
    popd
    exit /b 1
)
".venv\Scripts\python.exe" -B "%~dp0autonomy_web_launcher.py" wsl %*
set "TEAM_AUTONOMY_EXIT=%ERRORLEVEL%"
if not "%TEAM_AUTONOMY_EXIT%"=="0" pause
popd
exit /b %TEAM_AUTONOMY_EXIT%
