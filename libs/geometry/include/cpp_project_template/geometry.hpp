#pragma once

/**
 * @file geometry.hpp
 * @brief Main header for 3D geometry primitives and operations
 */

#include <string>
#include "cpp_project_template/geometry/pose3d.hpp"

namespace cpp_project_template {
namespace geometry {

/**
 * @brief Version information for the geometry library
 */
struct Version {
    static constexpr int major = 1;
    static constexpr int minor = 0;
    static constexpr int patch = 0;
    
    static std::string string() {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
};

} // namespace geometry
} // namespace cpp_project_template