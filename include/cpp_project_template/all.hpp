#pragma once

/**
 * @file all.hpp
 * @brief Umbrella header that includes all libraries in the project
 * 
 * This header provides convenient access to all functionality across
 * the Core, Geometry, and Robotics libraries.
 */

#include "cpp_project_template/core.hpp"
#include "cpp_project_template/geometry.hpp"
#include "cpp_project_template/robotics.hpp"
#include "cpp_project_template/config.hpp"
#include "cpp_project_template/integration.hpp"

namespace cpp_project_template {

/**
 * @brief Project-wide version information
 */
struct ProjectVersion {
    static constexpr int major = 1;
    static constexpr int minor = 0;
    static constexpr int patch = 0;
    
    static std::string string() {
        return std::to_string(major) + "." + std::to_string(minor) + "." + std::to_string(patch);
    }
    
    /**
     * @brief Get version information for all libraries
     */
    static std::string detailedString() {
        return "Project: " + string() + 
               ", Core: " + core::Version::string() +
               ", Geometry: " + geometry::Version::string() +
               ", Robotics: " + robotics::Version::string();
    }
};

} // namespace cpp_project_template