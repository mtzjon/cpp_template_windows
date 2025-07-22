@echo off
echo Quick CMake test...

if exist build rmdir /s /q build

echo Testing msvc-debug...
cmake --preset msvc-debug 2>error.log
if %errorlevel% neq 0 (
    echo FAILED - Check error.log
    type error.log
    exit /b 1
)

echo SUCCESS!
if exist error.log del error.log
pause