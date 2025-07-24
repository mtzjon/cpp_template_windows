#pragma once

/**
 * @file integration.hpp
 * @brief High-level integration APIs that combine multiple libraries
 * 
 * This header provides convenience functions and classes that integrate
 * functionality from multiple libraries to provide higher-level APIs.
 * This is a common use case for root include/ directory.
 */

#include "cpp_project_template/core.hpp"
#include "cpp_project_template/geometry.hpp"  
#include "cpp_project_template/robotics.hpp"
#include "cpp_project_template/config.hpp"

#include <vector>
#include <optional>
#include <functional>

namespace cpp_project_template {
namespace integration {

using namespace cpp_project_template::core::utilities;
using namespace cpp_project_template::geometry;
using namespace cpp_project_template::robotics;

/**
 * @brief High-level robot motion planner
 * 
 * Combines geometry and robotics libraries to provide
 * trajectory planning functionality.
 */
class MotionPlanner {
public:
    struct Waypoint {
        Pose3D pose;
        Twist velocity;
        double time;
        
        Waypoint(const Pose3D& p, const Twist& v, double t) 
            : pose(p), velocity(v), time(t) {}
    };
    
    struct Trajectory {
        std::vector<Waypoint> waypoints;
        double total_duration;
        
        bool empty() const { return waypoints.empty(); }
        size_t size() const { return waypoints.size(); }
        
        // Get pose at specific time using interpolation
        std::optional<Pose3D> getPoseAtTime(double t) const;
        
        // Get velocity at specific time using interpolation  
        std::optional<Twist> getVelocityAtTime(double t) const;
    };

    /**
     * @brief Plan a trajectory between poses
     * @param start_pose Starting pose
     * @param end_pose Target pose
     * @param max_linear_vel Maximum linear velocity
     * @param max_angular_vel Maximum angular velocity
     * @return Planned trajectory
     */
    static Trajectory planLinearTrajectory(
        const Pose3D& start_pose,
        const Pose3D& end_pose, 
        double max_linear_vel = 1.0,
        double max_angular_vel = 1.0
    );
    
    /**
     * @brief Plan trajectory through multiple waypoints
     * @param waypoints List of poses to visit
     * @param max_linear_vel Maximum linear velocity
     * @param max_angular_vel Maximum angular velocity
     * @return Planned trajectory
     */
    static Trajectory planWaypointTrajectory(
        const std::vector<Pose3D>& waypoints,
        double max_linear_vel = 1.0,
        double max_angular_vel = 1.0
    );
};

/**
 * @brief Force/torque analyzer for robotics applications
 * 
 * Combines robotics and geometry libraries for force analysis.
 */
class ForceAnalyzer {
public:
    struct ForcePoint {
        Pose3D location;
        Wrench force;
        
        ForcePoint(const Pose3D& loc, const Wrench& f) 
            : location(loc), force(f) {}
    };
    
    /**
     * @brief Calculate equivalent wrench at a reference point
     * @param forces List of forces at various locations
     * @param reference_point Reference point for wrench calculation
     * @return Equivalent wrench at reference point
     */
    static Wrench calculateEquivalentWrench(
        const std::vector<ForcePoint>& forces,
        const Pose3D& reference_point = Pose3D::Identity()
    );
    
    /**
     * @brief Check if forces are in equilibrium
     * @param forces List of forces
     * @param tolerance Tolerance for equilibrium check
     * @return True if forces are balanced
     */
    static bool isInEquilibrium(
        const std::vector<ForcePoint>& forces,
        double tolerance = config::DEFAULT_TOLERANCE
    );
    
    /**
     * @brief Calculate power consumption for a trajectory
     * @param trajectory Motion trajectory
     * @param external_forces External forces acting on the system
     * @return Total power consumption
     */
    static double calculatePowerConsumption(
        const MotionPlanner::Trajectory& trajectory,
        const std::vector<ForcePoint>& external_forces = {}
    );
};

/**
 * @brief Geometric utilities that combine core and geometry libraries
 */
namespace geometric_utils {
    
    /**
     * @brief Generate poses along a circle
     * @param center Center of the circle
     * @param radius Circle radius
     * @param normal Normal vector to the circle plane
     * @param num_points Number of points to generate
     * @param start_angle Starting angle in radians
     * @return Vector of poses along the circle
     */
    std::vector<Pose3D> generateCircularPath(
        const Eigen::Vector3d& center,
        double radius,
        const Eigen::Vector3d& normal = Eigen::Vector3d::UnitZ(),
        int num_points = 16,
        double start_angle = 0.0
    );
    
    /**
     * @brief Generate poses along a helical path
     * @param center Center of the helix
     * @param radius Helix radius
     * @param pitch Vertical distance per revolution
     * @param axis Helix axis direction
     * @param num_revolutions Number of complete revolutions
     * @param points_per_revolution Points per revolution
     * @return Vector of poses along the helix
     */
    std::vector<Pose3D> generateHelicalPath(
        const Eigen::Vector3d& center,
        double radius,
        double pitch,
        const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ(),
        double num_revolutions = 1.0,
        int points_per_revolution = 16
    );
    
    /**
     * @brief Create a coordinate frame from three points
     * @param origin Origin point
     * @param x_point Point defining X direction
     * @param xy_plane_point Point in XY plane
     * @return Pose representing the coordinate frame
     */
    Pose3D createFrameFromPoints(
        const Eigen::Vector3d& origin,
        const Eigen::Vector3d& x_point,
        const Eigen::Vector3d& xy_plane_point
    );
}

/**
 * @brief Robot kinematics utilities
 */
namespace kinematics {
    
    /**
     * @brief Calculate Jacobian matrix for velocity transformation
     * @param poses Chain of poses (e.g., robot joint poses)
     * @param end_effector_pose End effector pose
     * @return 6xN Jacobian matrix
     */
    Eigen::MatrixXd calculateJacobian(
        const std::vector<Pose3D>& poses,
        const Pose3D& end_effector_pose
    );
    
    /**
     * @brief Transform velocity through kinematic chain
     * @param joint_velocities Vector of joint velocities
     * @param jacobian Jacobian matrix
     * @return End effector twist
     */
    Twist transformVelocity(
        const Eigen::VectorXd& joint_velocities,
        const Eigen::MatrixXd& jacobian
    );
    
    /**
     * @brief Simple inverse kinematics solver (iterative)
     * @param target_pose Desired end effector pose
     * @param current_poses Current joint poses
     * @param max_iterations Maximum solver iterations
     * @param tolerance Convergence tolerance
     * @return Updated joint poses (empty if failed)
     */
    std::optional<std::vector<Pose3D>> solveInverseKinematics(
        const Pose3D& target_pose,
        const std::vector<Pose3D>& current_poses,
        int max_iterations = 100,
        double tolerance = config::DEFAULT_TOLERANCE
    );
}

} // namespace integration
} // namespace cpp_project_template