#include "cpp_project_template/robotics/twist.hpp"
#include <sstream>
#include <iomanip>

namespace cpp_project_template {
namespace robotics {

// Constructors
Twist::Twist() : linear_(Vector3d::Zero()), angular_(Vector3d::Zero()) {}

Twist::Twist(const Vector3d& linear, const Vector3d& angular)
    : linear_(linear), angular_(angular) {}

Twist::Twist(const Vector6d& twist_vector) {
    linear_ = twist_vector.head<3>();
    angular_ = twist_vector.tail<3>();
}

Twist::Twist(double vx, double vy, double vz, double wx, double wy, double wz)
    : linear_(vx, vy, vz), angular_(wx, wy, wz) {}

// Accessors
Twist::Vector6d Twist::vector() const {
    Vector6d result;
    result.head<3>() = linear_;
    result.tail<3>() = angular_;
    return result;
}

void Twist::setVector(const Vector6d& twist_vector) {
    linear_ = twist_vector.head<3>();
    angular_ = twist_vector.tail<3>();
}

// Operators
Twist Twist::operator+(const Twist& other) const {
    return Twist(linear_ + other.linear_, angular_ + other.angular_);
}

Twist Twist::operator-(const Twist& other) const {
    return Twist(linear_ - other.linear_, angular_ - other.angular_);
}

Twist Twist::operator*(double scalar) const {
    return Twist(linear_ * scalar, angular_ * scalar);
}

Twist Twist::operator/(double scalar) const {
    return Twist(linear_ / scalar, angular_ / scalar);
}

Twist Twist::operator-() const {
    return Twist(-linear_, -angular_);
}

Twist& Twist::operator+=(const Twist& other) {
    linear_ += other.linear_;
    angular_ += other.angular_;
    return *this;
}

Twist& Twist::operator-=(const Twist& other) {
    linear_ -= other.linear_;
    angular_ -= other.angular_;
    return *this;
}

Twist& Twist::operator*=(double scalar) {
    linear_ *= scalar;
    angular_ *= scalar;
    return *this;
}

Twist& Twist::operator/=(double scalar) {
    linear_ /= scalar;
    angular_ /= scalar;
    return *this;
}

bool Twist::isApprox(const Twist& other, double tolerance) const {
    return linear_.isApprox(other.linear_, tolerance) &&
           angular_.isApprox(other.angular_, tolerance);
}

// Twist operations
double Twist::norm() const {
    return vector().norm();
}

double Twist::squaredNorm() const {
    return vector().squaredNorm();
}

Twist Twist::normalized() const {
    double n = norm();
    if (n < 1e-12) return Twist::Zero();
    return (*this) / n;
}

void Twist::normalize() {
    double n = norm();
    if (n > 1e-12) {
        (*this) /= n;
    }
}

bool Twist::isZero(double tolerance) const {
    return norm() <= tolerance;
}

double Twist::dot(const Twist& other) const {
    return vector().dot(other.vector());
}

Twist::Vector3d Twist::angularCross(const Twist& other) const {
    return angular_.cross(other.angular_);
}

// Static factory methods
Twist Twist::Zero() {
    return Twist();
}

Twist Twist::Linear(const Vector3d& linear) {
    return Twist(linear, Vector3d::Zero());
}

Twist Twist::Angular(const Vector3d& angular) {
    return Twist(Vector3d::Zero(), angular);
}

Twist Twist::AxisVelocity(const Vector3d& axis, double linear_velocity, double angular_velocity) {
    Vector3d normalized_axis = axis.normalized();
    return Twist(normalized_axis * linear_velocity, normalized_axis * angular_velocity);
}

// Utility methods
std::string Twist::toString() const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "Linear: [" << linear_.x() << ", " << linear_.y() << ", " << linear_.z() << "], ";
    oss << "Angular: [" << angular_.x() << ", " << angular_.y() << ", " << angular_.z() << "]";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Twist& twist) {
    os << twist.toString();
    return os;
}

Twist operator*(double scalar, const Twist& twist) {
    return twist * scalar;
}

} // namespace robotics
} // namespace cpp_project_template