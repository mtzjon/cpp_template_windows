# C++ Project Template

A modern, production-ready C++ project template featuring the latest best practices, automated tooling, and comprehensive CI/CD integration for Windows development.

[![CI](https://github.com/yourorg/cpp-project-template/actions/workflows/ci.yml/badge.svg)](https://github.com/yourorg/cpp-project-template/actions/workflows/ci.yml)
[![Release](https://github.com/yourorg/cpp-project-template/actions/workflows/release.yml/badge.svg)](https://github.com/yourorg/cpp-project-template/actions/workflows/release.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://isocpp.org/)

## Features

### 🏗️ **Modern Build System**
- **CMake 3.21+** with modern practices
- **CMakePresets.json** for standardized build configurations
- **vcpkg** integration for dependency management
- **Cross-platform support** (Windows focus with Linux/macOS compatibility)

### 🔧 **Code Quality & Static Analysis**
- **clang-format** integration with comprehensive style configuration
- **clang-tidy** with extensive static analysis rules
- **Compiler warnings** as errors (MSVC `/W4`, GCC/Clang `-Wall -Wextra -Wpedantic`)
- **C++20** standard enforcement

### 🧪 **Testing Framework**
- **Catch2** and **Google Test** integration
- **Parameterized tests** and **fixtures**
- **Performance benchmarks**
- **CTest** integration with parallel execution

### 📚 **Documentation**
- **Doxygen** automatic documentation generation
- **GitHub Pages** deployment
- **Markdown support** with comprehensive README
- **API documentation** with code examples

### 🚀 **CI/CD Pipeline**
- **GitHub Actions** workflows for Windows
- **Multi-configuration builds** (Debug/Release)
- **Automated testing** on every PR
- **Security scanning** with CodeQL
- **Automatic releases** with package generation
- **Artifact caching** for faster builds

### 📦 **Package Management**
- **vcpkg** for C++ dependencies
- **FetchContent** for header-only libraries
- **CPack** for distribution packages (NSIS, ZIP)
- **CMake targets** for easy integration

### 👥 **Team Collaboration**
- **Git hooks** ready configuration
- **Editor config** for consistent formatting
- **Issue templates** and **PR templates**
- **Code review** guidelines

## Quick Start

### Prerequisites

**Windows:**
- Visual Studio 2022 or later
- CMake 3.21+
- Git
- vcpkg (optional, will be fetched automatically)

**Alternative (Cross-platform):**
- Modern C++20 compiler (GCC 11+, Clang 13+)
- CMake 3.21+
- Ninja build system

### Building the Project

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourorg/cpp-project-template.git
   cd cpp-project-template
   ```

2. **Configure and build using CMake presets:**
   ```bash
   # For Visual Studio (Windows)
   cmake --preset msvc-release
   cmake --build --preset msvc-release
   
   # For Ninja (Cross-platform)
   cmake --preset release
   cmake --build --preset release
   ```

3. **Run tests:**
   ```bash
   ctest --preset release
   ```

4. **Generate documentation:**
   ```bash
   cmake --build --preset release --target docs
   ```

### Usage Example

```cpp
#include "cpp_project_template/logger.hpp"

int main() {
    using namespace cpp_project_template;
    
    // Initialize the global logger
    initializeGlobalLogger("MyApp", Logger::Level::Info);
    auto& logger = getGlobalLogger();
    
    // Use modern C++ features
    logger.info("Hello, Modern C++!");
    logger.debug("Debug information: {}", 42);
    
    return 0;
}
```

## Project Structure

```
cpp-project-template/
├── .github/workflows/          # GitHub Actions CI/CD
├── cmake/                      # CMake modules and configs
├── docs/                       # Documentation configuration
├── include/cpp_project_template/  # Public headers
├── src/                        # Library source code
├── apps/                       # Executable applications
├── tests/                      # Unit and integration tests
├── CMakeLists.txt             # Main CMake configuration
├── CMakePresets.json          # Build presets
├── vcpkg.json                 # Package dependencies
├── .clang-format              # Code formatting rules
├── .clang-tidy                # Static analysis configuration
└── README.md                  # Project documentation
```

## Available CMake Presets

| Preset | Description | Generator |
|--------|-------------|-----------|
| `default` | Default Ninja build | Ninja |
| `debug` | Debug build with Ninja | Ninja |
| `release` | Release build with Ninja | Ninja |
| `msvc-debug` | MSVC Debug build | Visual Studio 2022 |
| `msvc-release` | MSVC Release build | Visual Studio 2022 |

## Dependencies

### Core Dependencies (Fetched Automatically)
- **fmt** - Modern formatting library
- **spdlog** - Fast logging library
- **CLI11** - Command line parser

### Testing Dependencies
- **Catch2** - Testing framework
- **Google Test/Mock** - Alternative testing framework

### Optional Features (vcpkg)
Enable features by configuring vcpkg:
- `testing` - Testing frameworks
- `networking` - HTTP and networking libraries
- `json` - JSON processing libraries
- `crypto` - Cryptographic libraries
- `gui` - GUI frameworks
- `database` - Database connectivity

## Code Quality Tools

### Formatting
```bash
# Format all source files
cmake --build --preset release --target clang-format
```

### Static Analysis
```bash
# Run clang-tidy
cmake --build build/release
find src apps tests -name '*.cpp' | xargs clang-tidy -p build/release
```

### Testing
```bash
# Run all tests
cmake --build --preset release
ctest --preset release --output-on-failure

# Run specific test suite
ctest --preset release -R "Logger"
```

## Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Code Standards
- Follow the existing code style (enforced by clang-format)
- Add tests for new functionality
- Update documentation as needed
- Ensure all CI checks pass

## Customization

### Renaming the Project
1. Update `CMakeLists.txt` project name and description
2. Rename the namespace in source files
3. Update include directory structure
4. Modify `vcpkg.json` metadata
5. Update documentation and README

### Adding Dependencies
1. **vcpkg dependencies:** Add to `vcpkg.json` features
2. **FetchContent:** Add to main `CMakeLists.txt`
3. **System packages:** Use `find_package()` in CMake

### Extending CI/CD
- Modify `.github/workflows/` for custom build steps
- Add deployment targets
- Configure additional static analysis tools
- Set up code coverage reporting

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **CMake** community for modern CMake practices
- **vcpkg** team for excellent package management
- **Catch2** and **Google Test** teams for testing frameworks
- **clang** team for excellent tooling support
- **GitHub Actions** for CI/CD infrastructure

---

**Ready to build something amazing? 🚀**

This template provides everything you need to start a professional C++ project with modern tooling and best practices. Fork it, customize it, and build your next great application!