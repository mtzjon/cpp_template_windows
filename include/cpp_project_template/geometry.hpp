#pragma once

/**
 * @file geometry.hpp
 * @brief Main header for robotics and geometry utilities based on Eigen
 * 
 * This library provides mathematical utilities for robotics, mechanics, and geometry
 * including 3D poses, twist vectors, wrench vectors, and other related utilities.
 */

#include <string>

#include "cpp_project_template/geometry/pose3d.hpp"
#include "cpp_project_template/geometry/twist.hpp"
#include "cpp_project_template/geometry/wrench.hpp"
#include "cpp_project_template/geometry/transforms.hpp"
#include "cpp_project_template/geometry/utilities.hpp"

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