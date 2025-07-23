#include "cpp_project_template/geometry/wrench.hpp"
#include "cpp_project_template/geometry/twist.hpp"
#include <sstream>
#include <iomanip>

namespace cpp_project_template {
namespace geometry {

// Constructors
Wrench::Wrench() : force_(Vector3d::Zero()), torque_(Vector3d::Zero()) {}

Wrench::Wrench(const Vector3d& force, const Vector3d& torque)
    : force_(force), torque_(torque) {}

Wrench::Wrench(const Vector6d& wrench_vector) {
    force_ = wrench_vector.head<3>();
    torque_ = wrench_vector.tail<3>();
}

Wrench::Wrench(double fx, double fy, double fz, double tx, double ty, double tz)
    : force_(fx, fy, fz), torque_(tx, ty, tz) {}

// Accessors
Wrench::Vector6d Wrench::vector() const {
    Vector6d result;
    result.head<3>() = force_;
    result.tail<3>() = torque_;
    return result;
}

void Wrench::setVector(const Vector6d& wrench_vector) {
    force_ = wrench_vector.head<3>();
    torque_ = wrench_vector.tail<3>();
}

// Operators
Wrench Wrench::operator+(const Wrench& other) const {
    return Wrench(force_ + other.force_, torque_ + other.torque_);
}

Wrench Wrench::operator-(const Wrench& other) const {
    return Wrench(force_ - other.force_, torque_ - other.torque_);
}

Wrench Wrench::operator*(double scalar) const {
    return Wrench(force_ * scalar, torque_ * scalar);
}

Wrench Wrench::operator/(double scalar) const {
    return Wrench(force_ / scalar, torque_ / scalar);
}

Wrench Wrench::operator-() const {
    return Wrench(-force_, -torque_);
}

Wrench& Wrench::operator+=(const Wrench& other) {
    force_ += other.force_;
    torque_ += other.torque_;
    return *this;
}

Wrench& Wrench::operator-=(const Wrench& other) {
    force_ -= other.force_;
    torque_ -= other.torque_;
    return *this;
}

Wrench& Wrench::operator*=(double scalar) {
    force_ *= scalar;
    torque_ *= scalar;
    return *this;
}

Wrench& Wrench::operator/=(double scalar) {
    force_ /= scalar;
    torque_ /= scalar;
    return *this;
}

bool Wrench::isApprox(const Wrench& other, double tolerance) const {
    return force_.isApprox(other.force_, tolerance) &&
           torque_.isApprox(other.torque_, tolerance);
}

// Wrench operations
double Wrench::norm() const {
    return vector().norm();
}

double Wrench::squaredNorm() const {
    return vector().squaredNorm();
}

Wrench Wrench::normalized() const {
    double n = norm();
    if (n < 1e-12) return Wrench::Zero();
    return (*this) / n;
}

void Wrench::normalize() {
    double n = norm();
    if (n > 1e-12) {
        (*this) /= n;
    }
}

bool Wrench::isZero(double tolerance) const {
    return norm() <= tolerance;
}

double Wrench::dot(const Wrench& other) const {
    return vector().dot(other.vector());
}

Wrench::Vector3d Wrench::forceCross(const Wrench& other) const {
    return force_.cross(other.force_);
}

Wrench::Vector3d Wrench::torqueCross(const Wrench& other) const {
    return torque_.cross(other.torque_);
}

double Wrench::power(const Twist& twist) const {
    return force_.dot(twist.linear()) + torque_.dot(twist.angular());
}

// Static factory methods
Wrench Wrench::Zero() {
    return Wrench();
}

Wrench Wrench::Force(const Vector3d& force) {
    return Wrench(force, Vector3d::Zero());
}

Wrench Wrench::Torque(const Vector3d& torque) {
    return Wrench(Vector3d::Zero(), torque);
}

Wrench Wrench::AxisForce(const Vector3d& axis, double force_magnitude, double torque_magnitude) {
    Vector3d normalized_axis = axis.normalized();
    return Wrench(normalized_axis * force_magnitude, normalized_axis * torque_magnitude);
}

Wrench Wrench::PointForce(const Vector3d& force, const Vector3d& point_of_application, 
                         const Vector3d& reference_point) {
    Vector3d moment_arm = point_of_application - reference_point;
    Vector3d torque = moment_arm.cross(force);
    return Wrench(force, torque);
}

// Utility methods
std::string Wrench::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Force: [" << force_.x() << ", " << force_.y() << ", " << force_.z() << "], ";
    oss << "Torque: [" << torque_.x() << ", " << torque_.y() << ", " << torque_.z() << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Wrench& wrench) {
    os << wrench.toString();
    return os;
}

Wrench operator*(double scalar, const Wrench& wrench) {
    return wrench * scalar;
}

} // namespace geometry
} // namespace cpp_project_template