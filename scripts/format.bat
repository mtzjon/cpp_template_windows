@echo off
echo Formatting C++ source files with clang-format...

where clang-format >nul 2>nul
if %errorlevel% neq 0 (
    echo ERROR: clang-format is not installed or not in PATH
    echo Please install clang-format
    pause
    exit /b 1
)

echo Found clang-format version:
clang-format --version

echo.
echo Formatting files...

for /r src %%f in (*.cpp *.hpp *.h) do (
    echo Formatting: %%f
    clang-format -i -style=file "%%f"
)

for /r apps %%f in (*.cpp *.hpp *.h) do (
    echo Formatting: %%f
    clang-format -i -style=file "%%f"
)

for /r tests %%f in (*.cpp *.hpp *.h) do (
    echo Formatting: %%f
    clang-format -i -style=file "%%f"
)

for /r include %%f in (*.cpp *.hpp *.h) do (
    echo Formatting: %%f
    clang-format -i -style=file "%%f"
)

echo.
echo Formatting completed!
pause