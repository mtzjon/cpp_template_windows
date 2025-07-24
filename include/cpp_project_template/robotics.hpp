#pragma once

/**
 * @file robotics.hpp
 * @brief Main header for robotics utilities including twists, wrenches, and transforms
 */

#include <string>
#include "cpp_project_template/robotics/twist.hpp"
#include "cpp_project_template/robotics/wrench.hpp"
#include "cpp_project_template/robotics/transforms.hpp"

namespace cpp_project_template {
namespace robotics {

/**
 * @brief Version information for the robotics library
 */
struct Version {
    static constexpr int major = 1;
    static constexpr int minor = 0;
    static constexpr int patch = 0;
    
    static std::string string() {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
};

} // namespace robotics
} // namespace cpp_project_template