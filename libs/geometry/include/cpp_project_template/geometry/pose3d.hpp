#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <string>

namespace cpp_project_template {
namespace geometry {

/**
 * @brief Represents a 3D pose combining position and orientation
 * 
 * A Pose3D consists of:
 * - Position: 3D vector (translation)
 * - Orientation: Unit quaternion (rotation)
 * 
 * This class provides operations for 3D pose manipulation commonly used
 * in robotics, computer vision, and mechanics.
 */
class Pose3D {
public:
    using Vector3d = Eigen::Vector3d;
    using Quaterniond = Eigen::Quaterniond;
    using Matrix4d = Eigen::Matrix4d;
    using Matrix3d = Eigen::Matrix3d;
    using AngleAxisd = Eigen::AngleAxisd;

    /**
     * @brief Default constructor - creates identity pose
     */
    Pose3D();

    /**
     * @brief Constructor from position and orientation
     * @param position 3D position vector
     * @param orientation Unit quaternion representing orientation
     */
    Pose3D(const Vector3d& position, const Quaterniond& orientation);

    /**
     * @brief Constructor from transformation matrix
     * @param transform 4x4 homogeneous transformation matrix
     */
    explicit Pose3D(const Matrix4d& transform);

    /**
     * @brief Constructor from position and Euler angles (ZYX convention)
     * @param position 3D position vector
     * @param roll Roll angle (rotation around X-axis) in radians
     * @param pitch Pitch angle (rotation around Y-axis) in radians
     * @param yaw Yaw angle (rotation around Z-axis) in radians
     */
    Pose3D(const Vector3d& position, double roll, double pitch, double yaw);

    // Accessors
    const Vector3d& position() const { return position_; }
    const Quaterniond& orientation() const { return orientation_; }
    
    Vector3d& position() { return position_; }
    Quaterniond& orientation() { return orientation_; }

    /**
     * @brief Get the 4x4 homogeneous transformation matrix
     * @return Transformation matrix representing this pose
     */
    Matrix4d matrix() const;

    /**
     * @brief Get the 3x3 rotation matrix
     * @return Rotation matrix from the quaternion
     */
    Matrix3d rotationMatrix() const;

    /**
     * @brief Get Euler angles (ZYX convention)
     * @return Vector3d containing [roll, pitch, yaw] in radians
     */
    Vector3d eulerAngles() const;

    // Operators
    /**
     * @brief Pose composition (this * other)
     * @param other Another pose to compose with this one
     * @return Composed pose
     */
    Pose3D operator*(const Pose3D& other) const;

    /**
     * @brief Transform a 3D point by this pose
     * @param point 3D point to transform
     * @return Transformed point
     */
    Vector3d operator*(const Vector3d& point) const;

    /**
     * @brief Pose composition assignment
     * @param other Pose to compose with this one
     * @return Reference to this pose after composition
     */
    Pose3D& operator*=(const Pose3D& other);

    /**
     * @brief Equality comparison with tolerance
     * @param other Pose to compare with
     * @param tolerance Tolerance for comparison (default: 1e-9)
     * @return True if poses are equal within tolerance
     */
    bool isApprox(const Pose3D& other, double tolerance = 1e-9) const;

    // Pose operations
    /**
     * @brief Get the inverse pose
     * @return Inverse of this pose
     */
    Pose3D inverse() const;

    /**
     * @brief Linear interpolation between two poses
     * @param other Target pose for interpolation
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated pose
     */
    Pose3D interpolate(const Pose3D& other, double t) const;

    /**
     * @brief Distance between two poses
     * @param other Other pose
     * @return Euclidean distance between positions
     */
    double distance(const Pose3D& other) const;

    /**
     * @brief Angular distance between orientations
     * @param other Other pose
     * @return Angular distance in radians
     */
    double angularDistance(const Pose3D& other) const;

    // Static factory methods
    /**
     * @brief Create identity pose
     * @return Identity pose (zero translation, no rotation)
     */
    static Pose3D Identity();

    /**
     * @brief Create pose from translation only
     * @param position 3D position vector
     * @return Pose with given translation and identity rotation
     */
    static Pose3D Translation(const Vector3d& position);

    /**
     * @brief Create pose from rotation only
     * @param orientation Unit quaternion
     * @return Pose with given rotation and zero translation
     */
    static Pose3D Rotation(const Quaterniond& orientation);

    /**
     * @brief Create pose from axis-angle rotation
     * @param axis Rotation axis (will be normalized)
     * @param angle Rotation angle in radians
     * @return Pose with given rotation and zero translation
     */
    static Pose3D AxisAngle(const Vector3d& axis, double angle);

    // Utility methods
    /**
     * @brief String representation of the pose
     * @return String containing position and orientation information
     */
    std::string toString() const;

    /**
     * @brief Stream output operator
     */
    friend std::ostream& operator<<(std::ostream& os, const Pose3D& pose);

private:
    Vector3d position_;      ///< 3D position vector
    Quaterniond orientation_; ///< Unit quaternion for orientation

    /**
     * @brief Normalize the quaternion to ensure it's unit
     */
    void normalizeQuaternion();
};

} // namespace geometry
} // namespace cpp_project_template