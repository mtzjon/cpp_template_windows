@echo off
echo Testing C++ Project Template Build...

echo.
echo Configuring with Ninja Release preset...
cmake --preset release
if %errorlevel% neq 0 (
    echo Configuration failed!
    exit /b 1
)

echo.
echo Building...
cmake --build --preset release
if %errorlevel% neq 0 (
    echo Build failed!
    exit /b 1
)

echo.
echo Running tests...
ctest --preset release --output-on-failure
if %errorlevel% neq 0 (
    echo Tests failed!
    exit /b 1
)

echo.
echo Build test completed successfully!
pause