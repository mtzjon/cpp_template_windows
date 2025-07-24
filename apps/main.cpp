#include "cpp_project_template/all.hpp"

#include <CLI/CLI.hpp>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace cpp_project_template;

/**
 * @brief Main application entry point
 *
 * Demonstrates the separated libraries: Core, Geometry, and Robotics
 */
int main(int argc, char* argv[]) {
    CLI::App app{"C++ Multi-Library Project - Demonstration of Core, Geometry, and Robotics libraries",
                 "multi-library-demo"};
    app.set_version_flag("--version", ProjectVersion::detailedString());

    // Command line options
    bool verbose = false;
    bool show_versions = false;
    std::string demo_type = "all";

    app.add_flag("-v,--verbose", verbose, "Enable verbose output");
    app.add_flag("--versions", show_versions, "Show library versions");
    app.add_option("-d,--demo", demo_type, "Demo type (all, core, geometry, robotics)")
        ->check(CLI::IsMember({"all", "core", "geometry", "robotics"}));

    CLI11_PARSE(app, argc, argv);

    try {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "=== Multi-Library Project Demo ===" << std::endl;
        
        if (show_versions) {
            std::cout << "Version Information:" << std::endl;
            std::cout << "  Project: " << ProjectVersion::string() << std::endl;
            std::cout << "  Core Library: " << core::Version::string() << std::endl;
            std::cout << "  Geometry Library: " << geometry::Version::string() << std::endl;
            std::cout << "  Robotics Library: " << robotics::Version::string() << std::endl;
            std::cout << std::endl;
        }

        if (demo_type == "core" || demo_type == "all") {
            std::cout << "=== Core Library Demo ===" << std::endl;
            
            // Mathematical utilities from core library
            using namespace core::utilities;
            
            std::cout << "Mathematical constants:" << std::endl;
            std::cout << "  PI = " << PI << std::endl;
            std::cout << "  45° in radians = " << degreesToRadians(45.0) << std::endl;
            
            // Angle normalization
            double angle = 450.0;  // degrees
            double normalized = normalizeAngle(degreesToRadians(angle));
            std::cout << "  " << angle << "° normalized = " << radiansToDegrees(normalized) << "°" << std::endl;
            
            // Vector operations
            Eigen::Vector3d v1(1, 2, 3);
            Eigen::Vector3d v2(4, 5, 6);
            std::cout << "Vector operations:" << std::endl;
            std::cout << "  v1 = " << v1.transpose() << std::endl;
            std::cout << "  v2 = " << v2.transpose() << std::endl;
            std::cout << "  Angle between = " << radiansToDegrees(angleBetween(v1, v2)) << "°" << std::endl;
            std::cout << "  Distance = " << distance(v1, v2) << std::endl;
            std::cout << std::endl;
        }

        if (demo_type == "geometry" || demo_type == "all") {
            std::cout << "=== Geometry Library Demo ===" << std::endl;
            
            // 3D Pose operations from geometry library
            using namespace geometry;
            
            Pose3D pose1(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
            Pose3D pose2 = Pose3D::AxisAngle(Eigen::Vector3d::UnitZ(), 
                                           core::utilities::degreesToRadians(45));
            
            std::cout << "3D Pose operations:" << std::endl;
            std::cout << "  Pose 1: " << pose1 << std::endl;
            std::cout << "  Pose 2: " << pose2 << std::endl;
            
            Pose3D composed = pose1 * pose2;
            std::cout << "  Composed: " << composed << std::endl;
            std::cout << "  Distance: " << pose1.distance(pose2) << std::endl;
            
            // Transform points
            Eigen::Vector3d point(0, 0, 1);
            Eigen::Vector3d transformed_point = pose1 * point;
            std::cout << "  Point " << point.transpose() << " -> " << transformed_point.transpose() << std::endl;
            std::cout << std::endl;
        }

        if (demo_type == "robotics" || demo_type == "all") {
            std::cout << "=== Robotics Library Demo ===" << std::endl;
            
            // Robotics operations from robotics library
            using namespace robotics;
            
            // Twist operations
            Twist twist1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));
            Twist twist2 = Twist::Angular(Eigen::Vector3d(0, 1, 0));
            
            std::cout << "Twist operations:" << std::endl;
            std::cout << "  Twist 1: " << twist1 << std::endl;
            std::cout << "  Twist 2: " << twist2 << std::endl;
            std::cout << "  Sum: " << (twist1 + twist2) << std::endl;
            std::cout << "  Twist 1 magnitude: " << twist1.norm() << std::endl;
            
            // Wrench operations
            Wrench wrench1 = Wrench::Force(Eigen::Vector3d(10, 0, 0));
            Wrench wrench2 = Wrench::PointForce(Eigen::Vector3d(0, 5, 0), 
                                               Eigen::Vector3d(1, 0, 0));
            
            std::cout << "Wrench operations:" << std::endl;
            std::cout << "  Force wrench: " << wrench1 << std::endl;
            std::cout << "  Point force wrench: " << wrench2 << std::endl;
            std::cout << "  Combined: " << (wrench1 + wrench2) << std::endl;
            
            // Power calculation (demonstrates interaction between libraries)
            double power = wrench1.power(twist1);
            std::cout << "  Power (F·v): " << power << " Watts" << std::endl;
            
            // Coordinate transformations (geometry + robotics)
            geometry::Pose3D robot_base = geometry::Pose3D::Translation(Eigen::Vector3d(5, 3, 0));
            Twist body_twist(Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0, 0, 0.5));
            Twist spatial_twist = transforms::transformTwist(body_twist, robot_base);
            
            std::cout << "Coordinate transformations:" << std::endl;
            std::cout << "  Body twist: " << body_twist << std::endl;
            std::cout << "  Spatial twist: " << spatial_twist << std::endl;
            std::cout << std::endl;
        }

        if (verbose) {
            std::cout << "=== Inter-Library Dependencies ===" << std::endl;
            std::cout << "Core Library:" << std::endl;
            std::cout << "  - Provides mathematical utilities" << std::endl;
            std::cout << "  - Used by both Geometry and Robotics libraries" << std::endl;
            std::cout << "Geometry Library:" << std::endl;
            std::cout << "  - Depends on Core" << std::endl;
            std::cout << "  - Provides 3D geometry primitives" << std::endl;
            std::cout << "Robotics Library:" << std::endl;
            std::cout << "  - Depends on Core and Geometry" << std::endl;
            std::cout << "  - Provides robotics-specific data structures" << std::endl;
            std::cout << "  - Implements transformations between coordinate frames" << std::endl;
        }

        std::cout << "=== Demo Complete ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}