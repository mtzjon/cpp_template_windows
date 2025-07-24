#include "cpp_project_template/geometry/pose3d.hpp"
#include <sstream>
#include <iomanip>

namespace cpp_project_template {
namespace geometry {

// Constructors
Pose3D::Pose3D() : position_(Vector3d::Zero()), orientation_(Quaterniond::Identity()) {}

Pose3D::Pose3D(const Vector3d& position, const Quaterniond& orientation)
    : position_(position), orientation_(orientation) {
    normalizeQuaternion();
}

Pose3D::Pose3D(const Matrix4d& transform) {
    position_ = transform.block<3, 1>(0, 3);
    Matrix3d rotation = transform.block<3, 3>(0, 0);
    orientation_ = Quaterniond(rotation);
    normalizeQuaternion();
}

Pose3D::Pose3D(const Vector3d& position, double roll, double pitch, double yaw)
    : position_(position) {
    // ZYX Euler angle convention
    orientation_ = Quaterniond(
        Eigen::AngleAxisd(yaw, Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Vector3d::UnitX())
    );
    normalizeQuaternion();
}

// Matrix conversions
Pose3D::Matrix4d Pose3D::matrix() const {
    Matrix4d transform = Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = orientation_.toRotationMatrix();
    transform.block<3, 1>(0, 3) = position_;
    return transform;
}

Pose3D::Matrix3d Pose3D::rotationMatrix() const {
    return orientation_.toRotationMatrix();
}

Pose3D::Vector3d Pose3D::eulerAngles() const {
    // Convert quaternion to ZYX Euler angles
    Matrix3d rotation = orientation_.toRotationMatrix();
    
    // Extract Euler angles (roll, pitch, yaw)
    double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    double pitch = std::asin(-rotation(2, 0));
    double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    
    return Vector3d(roll, pitch, yaw);
}

// Operators
Pose3D Pose3D::operator*(const Pose3D& other) const {
    Vector3d new_position = position_ + orientation_ * other.position_;
    Quaterniond new_orientation = orientation_ * other.orientation_;
    return Pose3D(new_position, new_orientation);
}

Pose3D::Vector3d Pose3D::operator*(const Vector3d& point) const {
    return orientation_ * point + position_;
}

Pose3D& Pose3D::operator*=(const Pose3D& other) {
    position_ = position_ + orientation_ * other.position_;
    orientation_ = orientation_ * other.orientation_;
    normalizeQuaternion();
    return *this;
}

bool Pose3D::isApprox(const Pose3D& other, double tolerance) const {
    return position_.isApprox(other.position_, tolerance) &&
           orientation_.isApprox(other.orientation_, tolerance);
}

// Pose operations
Pose3D Pose3D::inverse() const {
    Quaterniond inv_orientation = orientation_.conjugate();
    Vector3d inv_position = -(inv_orientation * position_);
    return Pose3D(inv_position, inv_orientation);
}

Pose3D Pose3D::interpolate(const Pose3D& other, double t) const {
    // Linear interpolation for position
    Vector3d interp_position = position_ + t * (other.position_ - position_);
    
    // Spherical linear interpolation for orientation
    Quaterniond interp_orientation = orientation_.slerp(t, other.orientation_);
    
    return Pose3D(interp_position, interp_orientation);
}

double Pose3D::distance(const Pose3D& other) const {
    return (position_ - other.position_).norm();
}

double Pose3D::angularDistance(const Pose3D& other) const {
    Quaterniond diff = orientation_.conjugate() * other.orientation_;
    return 2.0 * std::acos(std::abs(diff.w()));
}

// Static factory methods
Pose3D Pose3D::Identity() {
    return Pose3D();
}

Pose3D Pose3D::Translation(const Vector3d& position) {
    return Pose3D(position, Quaterniond::Identity());
}

Pose3D Pose3D::Rotation(const Quaterniond& orientation) {
    return Pose3D(Vector3d::Zero(), orientation);
}

Pose3D Pose3D::AxisAngle(const Vector3d& axis, double angle) {
    Vector3d normalized_axis = axis.normalized();
    Quaterniond rotation(AngleAxisd(angle, normalized_axis));
    return Pose3D(Vector3d::Zero(), rotation);
}

// Utility methods
std::string Pose3D::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Position: [" << position_.x() << ", " << position_.y() << ", " << position_.z() << "], ";
    oss << "Orientation: [" << orientation_.w() << ", " << orientation_.x() 
        << ", " << orientation_.y() << ", " << orientation_.z() << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Pose3D& pose) {
    os << pose.toString();
    return os;
}

// Private methods
void Pose3D::normalizeQuaternion() {
    orientation_.normalize();
}

} // namespace geometry
} // namespace cpp_project_template