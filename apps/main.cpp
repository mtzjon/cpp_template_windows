#include "cpp_project_template/geometry.hpp"

#include <CLI/CLI.hpp>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace cpp_project_template::geometry;

/**
 * @brief Main application entry point
 *
 * Demonstrates the robotics geometry library with poses, twists, wrenches,
 * and various geometric transformations.
 */
int main(int argc, char* argv[]) {
    CLI::App app{"C++ Robotics Geometry Library - Demonstration of Pose, Twist, and Wrench operations",
                 "robotics-geometry-demo"};
    app.set_version_flag("--version", "1.0.0");

    // Command line options
    bool verbose = false;
    bool interactive = false;
    std::string demo_type = "basic";

    app.add_flag("-v,--verbose", verbose, "Enable verbose output");
    app.add_flag("-i,--interactive", interactive, "Enable interactive mode");
    app.add_option("-d,--demo", demo_type, "Demo type (basic, transforms, robotics)")
        ->check(CLI::IsMember({"basic", "transforms", "robotics"}));

    CLI11_PARSE(app, argc, argv);

    try {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "=== Robotics Geometry Library Demo ===" << std::endl;
        std::cout << "Library Version: " << Version::string() << std::endl << std::endl;

        if (demo_type == "basic") {
            std::cout << "=== Basic Geometry Operations ===" << std::endl;

            // Pose3D demonstration
            std::cout << "\n--- 3D Pose Operations ---" << std::endl;
            Pose3D pose1(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
            Pose3D pose2 = Pose3D::AxisAngle(Eigen::Vector3d::UnitZ(), utilities::degreesToRadians(45));
            
            std::cout << "Pose 1: " << pose1 << std::endl;
            std::cout << "Pose 2: " << pose2 << std::endl;
            
            Pose3D composed = pose1 * pose2;
            std::cout << "Composed (P1 * P2): " << composed << std::endl;
            std::cout << "Distance between poses: " << pose1.distance(pose2) << std::endl;
            
            // Twist demonstration
            std::cout << "\n--- Twist (Velocity) Operations ---" << std::endl;
            Twist twist1(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, 1));  // Linear + Angular velocity
            Twist twist2 = Twist::Angular(Eigen::Vector3d(0, 1, 0));
            
            std::cout << "Twist 1: " << twist1 << std::endl;
            std::cout << "Twist 2: " << twist2 << std::endl;
            std::cout << "Sum: " << (twist1 + twist2) << std::endl;
            std::cout << "Twist 1 magnitude: " << twist1.norm() << std::endl;

            // Wrench demonstration
            std::cout << "\n--- Wrench (Force/Torque) Operations ---" << std::endl;
            Wrench wrench1 = Wrench::Force(Eigen::Vector3d(10, 0, 0));
            Wrench wrench2 = Wrench::PointForce(Eigen::Vector3d(0, 5, 0), 
                                               Eigen::Vector3d(1, 0, 0));  // Force at offset
            
            std::cout << "Force wrench: " << wrench1 << std::endl;
            std::cout << "Point force wrench: " << wrench2 << std::endl;
            std::cout << "Combined wrench: " << (wrench1 + wrench2) << std::endl;
            
            // Power calculation
            double power = wrench1.power(twist1);
            std::cout << "Power (wrench · twist): " << power << " Watts" << std::endl;

        } else if (demo_type == "transforms") {
            std::cout << "=== Transformation Operations ===" << std::endl;

            // Create a robot base frame
            Pose3D robot_base = Pose3D::Translation(Eigen::Vector3d(5, 3, 0));
            
            // Create an end-effector pose relative to base
            Pose3D ee_relative(Eigen::Vector3d(0.5, 0, 0.8), 
                              Eigen::Quaterniond(Eigen::AngleAxisd(utilities::degreesToRadians(30), 
                                                                  Eigen::Vector3d::UnitY())));
            
            // Transform to world coordinates
            Pose3D ee_world = transforms::transformPose(ee_relative, robot_base);
            
            std::cout << "Robot base: " << robot_base << std::endl;
            std::cout << "End-effector (relative): " << ee_relative << std::endl;
            std::cout << "End-effector (world): " << ee_world << std::endl;

            // Transform some points
            std::vector<Eigen::Vector3d> points = {
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector3d(1, 0, 0),
                Eigen::Vector3d(0, 1, 0),
                Eigen::Vector3d(0, 0, 1)
            };
            
            auto transformed_points = transforms::transformPoints(points, robot_base);
            
            std::cout << "\nTransformed points:" << std::endl;
            for (size_t i = 0; i < points.size(); ++i) {
                std::cout << "  " << points[i].transpose() << " -> " 
                         << transformed_points[i].transpose() << std::endl;
            }

            // Adjoint transformation for twists
            Twist body_twist(Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0, 0, 0.5));
            Twist spatial_twist = transforms::transformTwist(body_twist, robot_base);
            
            std::cout << "\nBody twist: " << body_twist << std::endl;
            std::cout << "Spatial twist: " << spatial_twist << std::endl;

        } else if (demo_type == "robotics") {
            std::cout << "=== Robotics Applications ===" << std::endl;

            // Simulate a simple robot trajectory
            std::vector<Pose3D> trajectory;
            
            // Create waypoints in a circular motion
            int num_points = 8;
            double radius = 2.0;
            double height = 1.0;
            
            for (int i = 0; i < num_points; ++i) {
                double angle = 2.0 * M_PI * i / num_points;
                Eigen::Vector3d position(radius * cos(angle), radius * sin(angle), height);
                
                // Orientation points toward center
                Pose3D waypoint = transforms::lookAt(position, Eigen::Vector3d::Zero(), 
                                                   Eigen::Vector3d::UnitZ());
                trajectory.push_back(waypoint);
            }
            
            std::cout << "Generated trajectory with " << trajectory.size() << " waypoints:" << std::endl;
            for (size_t i = 0; i < trajectory.size(); ++i) {
                std::cout << "  Waypoint " << i << ": pos=" 
                         << trajectory[i].position().transpose() << std::endl;
            }

            // Calculate velocities between waypoints
            std::cout << "\nEstimated velocities between waypoints:" << std::endl;
            double dt = 1.0;  // 1 second between waypoints
            
            for (size_t i = 0; i < trajectory.size() - 1; ++i) {
                Eigen::Vector3d linear_vel = (trajectory[i+1].position() - trajectory[i].position()) / dt;
                double angular_distance = trajectory[i].angularDistance(trajectory[i+1]);
                
                std::cout << "  " << i << "->" << (i+1) << ": linear=" 
                         << linear_vel.norm() << " m/s, angular=" 
                         << angular_distance/dt << " rad/s" << std::endl;
            }

            // Demonstrate force analysis
            std::cout << "\n--- Force Analysis ---" << std::endl;
            
            // Gravity wrench
            double mass = 10.0;  // kg
            Wrench gravity = Wrench::Force(Eigen::Vector3d(0, 0, -mass * 9.81));
            
            // Contact force at end-effector
            Wrench contact = Wrench::PointForce(Eigen::Vector3d(50, 0, 0),  // 50N in X
                                               Eigen::Vector3d(0, 0, 0.5),  // At 0.5m height
                                               Eigen::Vector3d::Zero());    // From origin
            
            Wrench total_wrench = gravity + contact;
            
            std::cout << "Gravity wrench: " << gravity << std::endl;
            std::cout << "Contact wrench: " << contact << std::endl;
            std::cout << "Total wrench: " << total_wrench << std::endl;
            std::cout << "Total force magnitude: " << total_wrench.force().norm() << " N" << std::endl;
            std::cout << "Total torque magnitude: " << total_wrench.torque().norm() << " Nm" << std::endl;
        }

        if (verbose) {
            std::cout << "\n=== Utility Functions Demo ===" << std::endl;
            
            // Angle utilities
            double angle_deg = 370.0;
            double angle_rad = utilities::degreesToRadians(angle_deg);
            double normalized = utilities::normalizeAngle(angle_rad);
            
            std::cout << "Angle: " << angle_deg << "° = " << angle_rad << " rad" << std::endl;
            std::cout << "Normalized: " << normalized << " rad = " 
                     << utilities::radiansToDegrees(normalized) << "°" << std::endl;
            
            // Vector utilities
            Eigen::Vector3d v1(1, 2, 3);
            Eigen::Vector3d v2(4, 5, 6);
            
            std::cout << "\nVector operations:" << std::endl;
            std::cout << "v1 = " << v1.transpose() << std::endl;
            std::cout << "v2 = " << v2.transpose() << std::endl;
            std::cout << "Angle between: " << utilities::radiansToDegrees(utilities::angleBetween(v1, v2)) << "°" << std::endl;
            std::cout << "Distance: " << utilities::distance(v1, v2) << std::endl;
        }

        std::cout << "\n=== Demo Complete ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown error occurred" << std::endl;
        return 1;
    }
}