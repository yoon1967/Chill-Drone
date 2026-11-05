@echo off
setlocal DisableDelayedExpansion
pushd "%~dp0.." || exit /b 1
call "%~dp0resolve_project_python.cmd"
if errorlevel 1 (
    pause
    popd
    exit /b 1
)
"%MAPPING_PROJECT_PYTHON%" -B "%~dp0..\infra\autonomy-web\launcher.py" wsl %*
set "TEAM_AUTONOMY_EXIT=%ERRORLEVEL%"
if not "%TEAM_AUTONOMY_EXIT%"=="0" pause
popd
exit /b %TEAM_AUTONOMY_EXIT%
