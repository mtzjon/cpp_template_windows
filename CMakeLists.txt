cmake_minimum_required(VERSION 3.20)

# Set the project name and version
project(
    CppProjectTemplate
    VERSION 1.0.0
    DESCRIPTION "A modern C++ project template with Eigen-based robotics and geometry utilities"
    LANGUAGES CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Global compiler-specific options
if(MSVC)
    add_compile_options(/W4 /permissive-)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(/O2)
    endif()
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O3)
    endif()
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-g -O0)
    endif()
endif()

# Include guards for multiple inclusions
include_guard(GLOBAL)

# Add custom CMake modules path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find vcpkg packages
find_package(Eigen3 REQUIRED)
find_package(CLI11 REQUIRED)

# Find optional packages for testing
find_package(Catch2 3 QUIET)
find_package(GTest QUIET)

# Documentation with Doxygen
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    # Configure Doxygen
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_MAN NO)
    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/docs")
    set(DOXYGEN_PROJECT_NAME "${PROJECT_NAME}")
    set(DOXYGEN_PROJECT_VERSION "${PROJECT_VERSION}")
    set(DOXYGEN_PROJECT_DESCRIPTION "${PROJECT_DESCRIPTION}")
    
    # Create documentation target
    doxygen_add_docs(
        docs
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/README.md
        COMMENT "Generating API documentation with Doxygen"
    )
else()
    message(STATUS "Doxygen not found, not building docs")
endif()

# Main library
add_subdirectory(src)

# Applications
add_subdirectory(apps)

# Testing
if(Catch2_FOUND AND BUILD_TESTING)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
    message(STATUS "Building with Catch2 tests")
endif()

if(GTest_FOUND AND BUILD_TESTING)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
    message(STATUS "Building with Google Test")
endif()

# Installation configuration
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install the library
install(TARGETS CppProjectTemplate
    EXPORT CppProjectTemplateTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Create and install package configuration files
configure_package_config_file(
    cmake/CppProjectTemplateConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/CppProjectTemplateConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CppProjectTemplate
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/CppProjectTemplateConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/CppProjectTemplateConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/CppProjectTemplateConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CppProjectTemplate
)

install(EXPORT CppProjectTemplateTargets
    FILE CppProjectTemplateTargets.cmake
    NAMESPACE CppProjectTemplate::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/CppProjectTemplate
)

# Print build configuration
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3 version: ${Eigen3_VERSION}")
if(Catch2_FOUND)
    message(STATUS "  Catch2 version: ${Catch2_VERSION}")
endif()
if(GTest_FOUND)
    message(STATUS "  GTest found: ${GTest_VERSION}")
endif()
message(STATUS "")
