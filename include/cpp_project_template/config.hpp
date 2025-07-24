#pragma once

/**
 * @file config.hpp
 * @brief Project-wide configuration and compile-time settings
 * 
 * This header provides compile-time configuration options that affect
 * all libraries in the project. It's included from the root include/
 * directory to serve as a central configuration point.
 */

#include <cstddef>
#include <string_view>

namespace cpp_project_template {
namespace config {

// Build configuration
#ifndef CPP_PROJECT_TEMPLATE_VERSION_MAJOR
#define CPP_PROJECT_TEMPLATE_VERSION_MAJOR 1
#endif

#ifndef CPP_PROJECT_TEMPLATE_VERSION_MINOR  
#define CPP_PROJECT_TEMPLATE_VERSION_MINOR 0
#endif

#ifndef CPP_PROJECT_TEMPLATE_VERSION_PATCH
#define CPP_PROJECT_TEMPLATE_VERSION_PATCH 0
#endif

// Compile-time features
#ifndef CPP_PROJECT_TEMPLATE_ENABLE_LOGGING
#define CPP_PROJECT_TEMPLATE_ENABLE_LOGGING 1
#endif

#ifndef CPP_PROJECT_TEMPLATE_ENABLE_ASSERTIONS
#define CPP_PROJECT_TEMPLATE_ENABLE_ASSERTIONS 1
#endif

#ifndef CPP_PROJECT_TEMPLATE_PRECISION
#define CPP_PROJECT_TEMPLATE_PRECISION double
#endif

// Numerical tolerances (used across all libraries)
constexpr double DEFAULT_TOLERANCE = 1e-9;
constexpr double ANGULAR_TOLERANCE = 1e-8;
constexpr double DISTANCE_TOLERANCE = 1e-10;

// Memory allocation limits
constexpr size_t MAX_TRAJECTORY_POINTS = 10000;
constexpr size_t MAX_MESH_VERTICES = 1000000;

// Performance settings
constexpr bool ENABLE_VECTORIZATION = true;
constexpr bool ENABLE_PARALLEL_PROCESSING = true;

// Debug settings
#ifdef NDEBUG
constexpr bool DEBUG_MODE = false;
#else
constexpr bool DEBUG_MODE = true;
#endif

/**
 * @brief Build information structure
 */
struct BuildInfo {
    static constexpr int version_major = CPP_PROJECT_TEMPLATE_VERSION_MAJOR;
    static constexpr int version_minor = CPP_PROJECT_TEMPLATE_VERSION_MINOR; 
    static constexpr int version_patch = CPP_PROJECT_TEMPLATE_VERSION_PATCH;
    
    static constexpr std::string_view compiler = 
#ifdef __clang__
        "Clang";
#elif defined(__GNUC__)
        "GCC";
#elif defined(_MSC_VER)
        "MSVC";
#else
        "Unknown";
#endif

    static constexpr std::string_view build_type =
#ifdef NDEBUG
        "Release";
#else
        "Debug";
#endif

    static std::string version_string() {
        return std::to_string(version_major) + "." + 
               std::to_string(version_minor) + "." + 
               std::to_string(version_patch);
    }
    
    static std::string full_info() {
        return std::string("v") + version_string() + 
               " (" + std::string(compiler) + " " + std::string(build_type) + ")";
    }
};

/**
 * @brief Feature flags that can be checked at runtime
 */
struct Features {
    static constexpr bool logging_enabled = CPP_PROJECT_TEMPLATE_ENABLE_LOGGING;
    static constexpr bool assertions_enabled = CPP_PROJECT_TEMPLATE_ENABLE_ASSERTIONS;
    static constexpr bool debug_mode = DEBUG_MODE;
    static constexpr bool vectorization = ENABLE_VECTORIZATION;
    static constexpr bool parallel_processing = ENABLE_PARALLEL_PROCESSING;
};

} // namespace config
} // namespace cpp_project_template

// Convenience macros for common operations
#if CPP_PROJECT_TEMPLATE_ENABLE_ASSERTIONS
#include <cassert>
#define CPP_PROJECT_ASSERT(condition, message) \
    assert((condition) && (message))
#else
#define CPP_PROJECT_ASSERT(condition, message) ((void)0)
#endif

#if CPP_PROJECT_TEMPLATE_ENABLE_LOGGING && !defined(NDEBUG)
#include <iostream>
#define CPP_PROJECT_DEBUG(message) \
    std::cout << "[DEBUG] " << __FILE__ << ":" << __LINE__ << " " << (message) << std::endl
#else
#define CPP_PROJECT_DEBUG(message) ((void)0)
#endif