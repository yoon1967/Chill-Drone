@echo off
rem Return a Python environment without changing the caller's checkout.
set "MAPPING_PROJECT_PYTHON=%~dp0..\.venv\Scripts\python.exe"
if exist "%MAPPING_PROJECT_PYTHON%" goto found
for /f "delims=" %%G in ('git -C "%~dp0.." rev-parse --path-format^=absolute --git-common-dir 2^>nul') do (
    if exist "%%G\..\.venv\Scripts\python.exe" set "MAPPING_PROJECT_PYTHON=%%G\..\.venv\Scripts\python.exe"
)
if exist "%MAPPING_PROJECT_PYTHON%" goto found
echo [ERROR] Project Python was not found in this checkout or the main Git worktree.
echo Prepare the Python environment described in apps\console\README.md.
set "MAPPING_PROJECT_PYTHON="
exit /b 1
:found
for %%P in ("%MAPPING_PROJECT_PYTHON%") do set "MAPPING_PROJECT_PYTHON=%%~fP"
echo Python: "%MAPPING_PROJECT_PYTHON%"
exit /b 0
