@echo off
echo Testing CMake configuration...

:: Clean any existing build
if exist build rmdir /s /q build >nul 2>nul

:: Test msvc-debug preset
echo Testing msvc-debug preset...
cmake --preset msvc-debug
if %errorlevel% neq 0 (
    echo FAIL: msvc-debug configuration failed
    exit /b 1
)

echo SUCCESS: msvc-debug configuration passed

:: Clean build directory
if exist build rmdir /s /q build >nul 2>nul

:: Test msvc-release preset
echo Testing msvc-release preset...
cmake --preset msvc-release
if %errorlevel% neq 0 (
    echo FAIL: msvc-release configuration failed
    exit /b 1
)

echo SUCCESS: msvc-release configuration passed

echo.
echo All CMake configurations passed successfully!
pause