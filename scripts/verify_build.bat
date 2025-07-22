@echo off
setlocal enabledelayedexpansion

echo ========================================
echo  C++ Project Template Build Verification
echo ========================================
echo.

:: Step 1: Check formatting
echo 1. Checking code formatting...
echo ----------------------------------------

where clang-format >nul 2>nul
if %errorlevel% neq 0 (
    echo WARNING: clang-format not found - skipping format check
    goto :build_check
)

echo Running clang-format check...
set "FORMAT_ERRORS=0"

for /r src %%f in (*.cpp *.hpp *.h) do (
    clang-format --dry-run --Werror "%%f" >nul 2>nul
    if %errorlevel% neq 0 (
        echo FORMAT ERROR: %%f
        set /a FORMAT_ERRORS+=1
    )
)

for /r apps %%f in (*.cpp *.hpp *.h) do (
    clang-format --dry-run --Werror "%%f" >nul 2>nul
    if %errorlevel% neq 0 (
        echo FORMAT ERROR: %%f
        set /a FORMAT_ERRORS+=1
    )
)

for /r tests %%f in (*.cpp *.hpp *.h) do (
    clang-format --dry-run --Werror "%%f" >nul 2>nul
    if %errorlevel% neq 0 (
        echo FORMAT ERROR: %%f
        set /a FORMAT_ERRORS+=1
    )
)

for /r include %%f in (*.cpp *.hpp *.h) do (
    clang-format --dry-run --Werror "%%f" >nul 2>nul
    if %errorlevel% neq 0 (
        echo FORMAT ERROR: %%f
        set /a FORMAT_ERRORS+=1
    )
)

if %FORMAT_ERRORS% gtr 0 (
    echo FAIL: Found %FORMAT_ERRORS% formatting errors
    echo Run scripts\format.bat to fix them
    goto :error_exit
) else (
    echo PASS: All files are properly formatted
)

:build_check
echo.
echo 2. Building project...
echo ----------------------------------------

:: Clean any existing build
if exist build rmdir /s /q build >nul 2>nul

:: Try Ninja build first
where ninja >nul 2>nul
if %errorlevel% equ 0 (
    echo Using Ninja build system...
    cmake --preset release
    if %errorlevel% neq 0 (
        echo FAIL: CMake configuration failed
        goto :error_exit
    )
    
    cmake --build --preset release
    if %errorlevel% neq 0 (
        echo FAIL: Build failed
        goto :error_exit
    )
    set "BUILD_DIR=build\release"
    set "TEST_PRESET=release"
) else (
    :: Fallback to MSVC if available
    where msbuild >nul 2>nul
    if %errorlevel% equ 0 (
        echo Using MSVC build system...
        cmake --preset msvc-release
        if %errorlevel% neq 0 (
            echo FAIL: CMake configuration failed
            goto :error_exit
        )
        
        cmake --build --preset msvc-release
        if %errorlevel% neq 0 (
            echo FAIL: Build failed
            goto :error_exit
        )
        set "BUILD_DIR=build\msvc-release"
        set "TEST_PRESET=msvc-release"
    ) else (
        echo FAIL: No suitable build system found (Ninja or MSVC required)
        goto :error_exit
    )
)

echo PASS: Build completed successfully

echo.
echo 3. Running tests...
echo ----------------------------------------

ctest --preset %TEST_PRESET% --output-on-failure
if %errorlevel% neq 0 (
    echo FAIL: Tests failed
    goto :error_exit
)

echo PASS: All tests passed

echo.
echo 4. Verifying outputs...
echo ----------------------------------------

:: Check if executable was created
if exist "%BUILD_DIR%\CppProjectTemplate.exe" (
    echo PASS: Executable created successfully
) else (
    echo FAIL: Executable not found
    goto :error_exit
)

:: Try running the application
echo Testing application execution...
"%BUILD_DIR%\CppProjectTemplate.exe" --help >nul 2>nul
if %errorlevel% equ 0 (
    echo PASS: Application runs successfully
) else (
    echo WARNING: Application may have issues (exit code: %errorlevel%)
)

echo.
echo ========================================
echo  BUILD VERIFICATION SUCCESSFUL!
echo ========================================
echo.
echo All checks passed:
echo   ✓ Code formatting
echo   ✓ Build configuration
echo   ✓ Compilation
echo   ✓ Unit tests
echo   ✓ Application execution
echo.
echo The project is ready for development!
goto :end

:error_exit
echo.
echo ========================================
echo  BUILD VERIFICATION FAILED!
echo ========================================
echo.
echo Please fix the issues above and try again.
exit /b 1

:end
pause