@echo off
setlocal enabledelayedexpansion

echo ========================================
echo  C++ Project Template Setup Script
echo ========================================
echo.

:: Check if CMake is installed
where cmake >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: CMake is not installed or not in PATH
    echo Please install CMake 3.21 or later
    pause
    exit /b 1
)

:: Check CMake version
echo Checking CMake version...
cmake --version | findstr /C:"cmake version"

:: Check if Visual Studio is available
where msbuild >nul 2>nul
if %errorlevel% equ 0 (
    echo Visual Studio Build Tools found
    set "HAS_MSBUILD=1"
) else (
    echo Visual Studio Build Tools not found
    set "HAS_MSBUILD=0"
)

:: Check if Ninja is available
where ninja >nul 2>nul
if %errorlevel% equ 0 (
    echo Ninja build system found
    set "HAS_NINJA=1"
) else (
    echo Ninja not found - installing via chocolatey...
    where choco >nul 2>nul
    if %errorlevel% equ 0 (
        choco install ninja -y
        set "HAS_NINJA=1"
    ) else (
        echo Chocolatey not found - please install Ninja manually
        set "HAS_NINJA=0"
    )
)

echo.
echo Available build configurations:

if "%HAS_MSBUILD%"=="1" (
    echo   1. Visual Studio 2022 ^(msvc-release^)
    echo   2. Visual Studio 2022 Debug ^(msvc-debug^)
)

if "%HAS_NINJA%"=="1" (
    echo   3. Ninja Release ^(release^)
    echo   4. Ninja Debug ^(debug^)
)

echo   5. List all available presets
echo   0. Exit
echo.

set /p choice="Choose build configuration (1-5, 0 to exit): "

if "%choice%"=="0" goto :end
if "%choice%"=="5" goto :list_presets

if "%choice%"=="1" (
    if "%HAS_MSBUILD%"=="1" (
        set "PRESET=msvc-release"
        goto :configure
    ) else (
        echo Visual Studio not available
        goto :choose_again
    )
)

if "%choice%"=="2" (
    if "%HAS_MSBUILD%"=="1" (
        set "PRESET=msvc-debug"
        goto :configure
    ) else (
        echo Visual Studio not available
        goto :choose_again
    )
)

if "%choice%"=="3" (
    if "%HAS_NINJA%"=="1" (
        set "PRESET=release"
        goto :configure
    ) else (
        echo Ninja not available
        goto :choose_again
    )
)

if "%choice%"=="4" (
    if "%HAS_NINJA%"=="1" (
        set "PRESET=debug"
        goto :configure
    ) else (
        echo Ninja not available
        goto :choose_again
    )
)

echo Invalid choice
:choose_again
echo.
pause
goto :end

:list_presets
echo.
echo Available CMake presets:
cmake --list-presets=all
echo.
pause
goto :end

:configure
echo.
echo Configuring with preset: %PRESET%
echo ========================================

cmake --preset %PRESET%
if %errorlevel% neq 0 (
    echo Configuration failed!
    pause
    exit /b 1
)

echo.
echo Building project...
echo ========================================

cmake --build --preset %PRESET%
if %errorlevel% neq 0 (
    echo Build failed!
    pause
    exit /b 1
)

echo.
echo Running tests...
echo ========================================

ctest --preset %PRESET% --output-on-failure
if %errorlevel% neq 0 (
    echo Some tests failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo  Setup completed successfully!
echo ========================================
echo.
echo You can now:
echo   - Run the application: build\%PRESET%\CppProjectTemplate.exe
echo   - Generate docs: cmake --build --preset %PRESET% --target docs
echo   - Format code: cmake --build --preset %PRESET% --target clang-format
echo.

:end
pause
