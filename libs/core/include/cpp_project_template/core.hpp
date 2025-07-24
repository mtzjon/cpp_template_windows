#pragma once

/**
 * @file core.hpp
 * @brief Main header for core mathematical and utility functions
 */

#include "cpp_project_template/core/utilities.hpp"

namespace cpp_project_template {
namespace core {

/**
 * @brief Version information for the core library
 */
struct Version {
    static constexpr int major = 1;
    static constexpr int minor = 0;
    static constexpr int patch = 0;
    
    static std::string string() {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
};

} // namespace core
} // namespace cpp_project_template