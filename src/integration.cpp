#include "cpp_project_template/integration.hpp"
#include "cpp_project_template/config.hpp"

#include <algorithm>
#include <cmath>

namespace cpp_project_template {
namespace integration {

// MotionPlanner::Trajectory implementation

std::optional<Pose3D> MotionPlanner::Trajectory::getPoseAtTime(double t) const {
    if (waypoints.empty() || t < 0.0 || t > total_duration) {
        return std::nullopt;
    }
    
    // Find the segment containing time t
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
            double segment_duration = waypoints[i + 1].time - waypoints[i].time;
            double segment_ratio = (t - waypoints[i].time) / segment_duration;
            
            // Linear interpolation for position, SLERP for orientation
            return waypoints[i].pose.interpolate(waypoints[i + 1].pose, segment_ratio);
        }
    }
    
    // If we reach here, return the last waypoint
    return waypoints.back().pose;
}

std::optional<Twist> MotionPlanner::Trajectory::getVelocityAtTime(double t) const {
    if (waypoints.empty() || t < 0.0 || t > total_duration) {
        return std::nullopt;
    }
    
    // Find the segment containing time t
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        if (t >= waypoints[i].time && t <= waypoints[i + 1].time) {
            double segment_duration = waypoints[i + 1].time - waypoints[i].time;
            double segment_ratio = (t - waypoints[i].time) / segment_duration;
            
            // Linear interpolation of velocities
            Twist v1 = waypoints[i].velocity;
            Twist v2 = waypoints[i + 1].velocity;
            
            return Twist(
                lerp(v1.linear(), v2.linear(), segment_ratio),
                lerp(v1.angular(), v2.angular(), segment_ratio)
            );
        }
    }
    
    // If we reach here, return the last waypoint velocity
    return waypoints.back().velocity;
}

// MotionPlanner implementation

MotionPlanner::Trajectory MotionPlanner::planLinearTrajectory(
    const Pose3D& start_pose,
    const Pose3D& end_pose, 
    double max_linear_vel,
    double max_angular_vel
) {
    Trajectory trajectory;
    
    // Calculate required time based on distance and velocity limits
    double linear_distance = start_pose.distance(end_pose);
    double angular_distance = start_pose.angularDistance(end_pose);
    
    double linear_time = linear_distance / max_linear_vel;
    double angular_time = angular_distance / max_angular_vel;
    double total_time = std::max(linear_time, angular_time);
    
    // Create waypoints
    Eigen::Vector3d linear_vel = (end_pose.position() - start_pose.position()) / total_time;
    Eigen::Vector3d angular_vel_axis;
    double angular_vel_magnitude;
    
    // Simple angular velocity calculation (this could be improved)
    if (angular_distance > config::ANGULAR_TOLERANCE) {
        angular_vel_magnitude = angular_distance / total_time;
        // For simplicity, assume rotation around Z-axis
        angular_vel_axis = Eigen::Vector3d::UnitZ() * angular_vel_magnitude;
    } else {
        angular_vel_axis = Eigen::Vector3d::Zero();
    }
    
    Twist start_velocity(linear_vel, angular_vel_axis);
    Twist end_velocity = Twist::Zero(); // Come to rest
    
    trajectory.waypoints.emplace_back(start_pose, start_velocity, 0.0);
    trajectory.waypoints.emplace_back(end_pose, end_velocity, total_time);
    trajectory.total_duration = total_time;
    
    return trajectory;
}

MotionPlanner::Trajectory MotionPlanner::planWaypointTrajectory(
    const std::vector<Pose3D>& waypoints,
    double max_linear_vel,
    double max_angular_vel
) {
    Trajectory trajectory;
    
    if (waypoints.size() < 2) {
        return trajectory; // Empty trajectory
    }
    
    double current_time = 0.0;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        auto segment = planLinearTrajectory(
            waypoints[i], waypoints[i + 1], 
            max_linear_vel, max_angular_vel
        );
        
        // Add waypoints with adjusted time
        for (const auto& wp : segment.waypoints) {
            trajectory.waypoints.emplace_back(
                wp.pose, wp.velocity, current_time + wp.time
            );
        }
        
        current_time += segment.total_duration;
    }
    
    trajectory.total_duration = current_time;
    return trajectory;
}

// ForceAnalyzer implementation

Wrench ForceAnalyzer::calculateEquivalentWrench(
    const std::vector<ForcePoint>& forces,
    const Pose3D& reference_point
) {
    Eigen::Vector3d total_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d total_torque = Eigen::Vector3d::Zero();
    
    for (const auto& force_point : forces) {
        // Transform force to reference point
        Wrench transformed_wrench = transforms::transformWrench(
            force_point.force, 
            reference_point.inverse() * force_point.location
        );
        
        total_force += transformed_wrench.force();
        total_torque += transformed_wrench.torque();
    }
    
    return Wrench(total_force, total_torque);
}

bool ForceAnalyzer::isInEquilibrium(
    const std::vector<ForcePoint>& forces,
    double tolerance
) {
    Wrench total_wrench = calculateEquivalentWrench(forces);
    
    return total_wrench.force().norm() < tolerance && 
           total_wrench.torque().norm() < tolerance;
}

double ForceAnalyzer::calculatePowerConsumption(
    const MotionPlanner::Trajectory& trajectory,
    const std::vector<ForcePoint>& external_forces
) {
    if (trajectory.empty()) {
        return 0.0;
    }
    
    // Simple numerical integration
    double total_power = 0.0;
    double dt = 0.01; // 10ms time step
    
    for (double t = 0.0; t < trajectory.total_duration; t += dt) {
        auto velocity = trajectory.getVelocityAtTime(t);
        if (!velocity.has_value()) continue;
        
        // Calculate total external wrench at this time
        Wrench total_external = Wrench::Zero();
        for (const auto& force_point : external_forces) {
            total_external += force_point.force;
        }
        
        // Power = Force · Velocity
        double instantaneous_power = total_external.power(velocity.value());
        total_power += instantaneous_power * dt;
    }
    
    return total_power;
}

// geometric_utils implementation

namespace geometric_utils {

std::vector<Pose3D> generateCircularPath(
    const Eigen::Vector3d& center,
    double radius,
    const Eigen::Vector3d& normal,
    int num_points,
    double start_angle
) {
    std::vector<Pose3D> poses;
    poses.reserve(num_points);
    
    // Create orthonormal basis
    Eigen::Vector3d n = normal.normalized();
    Eigen::Vector3d u = (std::abs(n.dot(Eigen::Vector3d::UnitX())) < 0.9) ? 
                        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    u = (u - u.dot(n) * n).normalized(); // Gram-Schmidt
    Eigen::Vector3d v = n.cross(u);
    
    for (int i = 0; i < num_points; ++i) {
        double angle = start_angle + 2.0 * M_PI * i / num_points;
        
        Eigen::Vector3d position = center + 
            radius * (std::cos(angle) * u + std::sin(angle) * v);
        
        // Orientation: look toward center, up along normal
        Eigen::Vector3d forward = (center - position).normalized();
        Eigen::Vector3d right = forward.cross(n).normalized();
        Eigen::Vector3d up = right.cross(forward);
        
        Eigen::Matrix3d rotation;
        rotation.col(0) = right;
        rotation.col(1) = up;
        rotation.col(2) = -forward; // Negative for right-handed system
        
        poses.emplace_back(position, Eigen::Quaterniond(rotation));
    }
    
    return poses;
}

std::vector<Pose3D> generateHelicalPath(
    const Eigen::Vector3d& center,
    double radius,
    double pitch,
    const Eigen::Vector3d& axis,
    double num_revolutions,
    int points_per_revolution
) {
    std::vector<Pose3D> poses;
    int total_points = static_cast<int>(num_revolutions * points_per_revolution);
    poses.reserve(total_points);
    
    // Create orthonormal basis
    Eigen::Vector3d n = axis.normalized();
    Eigen::Vector3d u = (std::abs(n.dot(Eigen::Vector3d::UnitX())) < 0.9) ? 
                        Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    u = (u - u.dot(n) * n).normalized();
    Eigen::Vector3d v = n.cross(u);
    
    for (int i = 0; i < total_points; ++i) {
        double t = static_cast<double>(i) / points_per_revolution; // revolutions
        double angle = 2.0 * M_PI * t;
        
        Eigen::Vector3d position = center + 
            radius * (std::cos(angle) * u + std::sin(angle) * v) +
            (pitch * t) * n;
        
        // Orientation: tangent to helix
        Eigen::Vector3d tangent = 
            radius * (-std::sin(angle) * u + std::cos(angle) * v) +
            pitch * n;
        tangent.normalize();
        
        Eigen::Vector3d normal_dir = n;
        Eigen::Vector3d binormal = tangent.cross(normal_dir).normalized();
        normal_dir = binormal.cross(tangent);
        
        Eigen::Matrix3d rotation;
        rotation.col(0) = tangent;
        rotation.col(1) = binormal;
        rotation.col(2) = normal_dir;
        
        poses.emplace_back(position, Eigen::Quaterniond(rotation));
    }
    
    return poses;
}

Pose3D createFrameFromPoints(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& x_point,
    const Eigen::Vector3d& xy_plane_point
) {
    Eigen::Vector3d x_axis = (x_point - origin).normalized();
    Eigen::Vector3d y_temp = xy_plane_point - origin;
    
    // Ensure Y is perpendicular to X
    Eigen::Vector3d y_axis = (y_temp - y_temp.dot(x_axis) * x_axis).normalized();
    Eigen::Vector3d z_axis = x_axis.cross(y_axis);
    
    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    
    return Pose3D(origin, Eigen::Quaterniond(rotation));
}

} // namespace geometric_utils

// kinematics implementation (basic implementations)

namespace kinematics {

Eigen::MatrixXd calculateJacobian(
    const std::vector<Pose3D>& poses,
    const Pose3D& end_effector_pose
) {
    // This is a simplified implementation
    // Real robots would have more complex kinematics
    
    int n_joints = static_cast<int>(poses.size());
    Eigen::MatrixXd jacobian(6, n_joints);
    jacobian.setZero();
    
    for (int i = 0; i < n_joints; ++i) {
        // Simplified: assume revolute joints around Z-axis
        Eigen::Vector3d joint_axis = poses[i].rotationMatrix().col(2);
        Eigen::Vector3d joint_to_ee = end_effector_pose.position() - poses[i].position();
        
        // Linear velocity contribution
        Eigen::Vector3d linear_contrib = joint_axis.cross(joint_to_ee);
        jacobian.block<3, 1>(0, i) = linear_contrib;
        
        // Angular velocity contribution
        jacobian.block<3, 1>(3, i) = joint_axis;
    }
    
    return jacobian;
}

Twist transformVelocity(
    const Eigen::VectorXd& joint_velocities,
    const Eigen::MatrixXd& jacobian
) {
    if (jacobian.cols() != joint_velocities.size()) {
        CPP_PROJECT_DEBUG("Jacobian and joint velocity dimensions don't match");
        return Twist::Zero();
    }
    
    Eigen::VectorXd ee_velocity = jacobian * joint_velocities;
    
    return Twist(
        ee_velocity.head<3>(),  // Linear velocity
        ee_velocity.tail<3>()   // Angular velocity
    );
}

std::optional<std::vector<Pose3D>> solveInverseKinematics(
    const Pose3D& target_pose,
    const std::vector<Pose3D>& current_poses,
    int max_iterations,
    double tolerance
) {
    // This is a very simplified iterative IK solver
    // Real implementations would be much more sophisticated
    
    std::vector<Pose3D> poses = current_poses;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Calculate forward kinematics (simplified)
        Pose3D current_ee = poses.empty() ? Pose3D::Identity() : poses.back();
        
        // Check convergence
        double position_error = current_ee.distance(target_pose);
        double orientation_error = current_ee.angularDistance(target_pose);
        
        if (position_error < tolerance && orientation_error < tolerance) {
            return poses; // Converged
        }
        
        // Simple gradient descent update (very basic)
        // In practice, you'd use Jacobian-based methods
        for (size_t i = 0; i < poses.size(); ++i) {
            // Small random perturbation (placeholder for real IK)
            double delta = 0.01 * (2.0 * rand() / RAND_MAX - 1.0);
            Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
            poses[i] = poses[i] * Pose3D::AxisAngle(axis, delta);
        }
    }
    
    // Failed to converge
    return std::nullopt;
}

} // namespace kinematics

} // namespace integration
} // namespace cpp_project_template