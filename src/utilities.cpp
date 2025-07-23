#include "cpp_project_template/geometry/utilities.hpp"
#include <cmath>
#include <algorithm>

namespace cpp_project_template {
namespace geometry {
namespace utilities {

// Angle utilities
double normalizeAngle(double angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

double normalizeAnglePositive(double angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0) angle += TWO_PI;
    return angle;
}

double angularDistance(double from, double to) {
    return normalizeAngle(to - from);
}

// Vector utilities
bool isZero(const Vector3d& vector, double tolerance) {
    return vector.norm() <= tolerance;
}

Vector3d safeNormalize(const Vector3d& vector, double tolerance) {
    double norm = vector.norm();
    if (norm <= tolerance) {
        return Vector3d::Zero();
    }
    return vector / norm;
}

double angleBetween(const Vector3d& v1, const Vector3d& v2) {
    double dot_product = v1.normalized().dot(v2.normalized());
    return std::acos(std::clamp(dot_product, -1.0, 1.0));
}

Vector3d projectOnto(const Vector3d& vector, const Vector3d& onto) {
    return onto * (vector.dot(onto) / onto.squaredNorm());
}

Vector3d perpendicularComponent(const Vector3d& vector, const Vector3d& direction) {
    return vector - projectOnto(vector, direction);
}

Matrix3d createOrthonormalBasis(const Vector3d& z_axis) {
    Vector3d z = z_axis.normalized();
    
    // Find a vector that's not parallel to z
    Vector3d temp = (std::abs(z.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
    
    Vector3d x = z.cross(temp).normalized();
    Vector3d y = z.cross(x);
    
    Matrix3d basis;
    basis.col(0) = x;
    basis.col(1) = y;
    basis.col(2) = z;
    return basis;
}

// Rotation utilities
Matrix3d axisAngleToMatrix(const Vector3d& axis, double angle) {
    return AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

void matrixToAxisAngle(const Matrix3d& rotation, Vector3d& axis, double& angle) {
    AngleAxisd aa(rotation);
    axis = aa.axis();
    angle = aa.angle();
}

Matrix3d eulerToMatrix(double roll, double pitch, double yaw) {
    return (AngleAxisd(yaw, Vector3d::UnitZ()) *
            AngleAxisd(pitch, Vector3d::UnitY()) *
            AngleAxisd(roll, Vector3d::UnitX())).toRotationMatrix();
}

Vector3d matrixToEuler(const Matrix3d& rotation) {
    // Extract Euler angles (ZYX convention)
    double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    double pitch = std::asin(-rotation(2, 0));
    double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return Vector3d(roll, pitch, yaw);
}

Matrix3d rotationBetween(const Vector3d& from, const Vector3d& to) {
    Vector3d v1 = from.normalized();
    Vector3d v2 = to.normalized();
    
    Vector3d cross_product = v1.cross(v2);
    double dot_product = v1.dot(v2);
    
    if (cross_product.norm() < 1e-6) {
        // Vectors are parallel or anti-parallel
        if (dot_product > 0) {
            return Matrix3d::Identity();
        } else {
            return -Matrix3d::Identity();
        }
    }
    
    double angle = std::acos(std::clamp(dot_product, -1.0, 1.0));
    return axisAngleToMatrix(cross_product, angle);
}

// Skew-symmetric matrix utilities
Matrix3d skewSymmetric(const Vector3d& vector) {
    Matrix3d skew;
    skew << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;
    return skew;
}

Vector3d unskewSymmetric(const Matrix3d& skew_matrix) {
    return Vector3d(skew_matrix(2, 1), skew_matrix(0, 2), skew_matrix(1, 0));
}

// Statistics utilities
Vector3d mean(const std::vector<Vector3d>& points) {
    if (points.empty()) return Vector3d::Zero();
    
    Vector3d sum = Vector3d::Zero();
    for (const auto& point : points) {
        sum += point;
    }
    return sum / static_cast<double>(points.size());
}

Matrix3d covariance(const std::vector<Vector3d>& points, const Vector3d& mean_point) {
    if (points.empty()) return Matrix3d::Zero();
    
    Vector3d actual_mean = (mean_point == Vector3d::Zero()) ? mean(points) : mean_point;
    
    Matrix3d cov = Matrix3d::Zero();
    for (const auto& point : points) {
        Vector3d diff = point - actual_mean;
        cov += diff * diff.transpose();
    }
    
    return cov / static_cast<double>(points.size() - 1);
}

// Distance metrics
double manhattanDistance(const Vector3d& p1, const Vector3d& p2) {
    Vector3d diff = p1 - p2;
    return std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z());
}

double chebyshevDistance(const Vector3d& p1, const Vector3d& p2) {
    Vector3d diff = (p1 - p2).cwiseAbs();
    return diff.maxCoeff();
}

} // namespace utilities
} // namespace geometry
} // namespace cpp_project_template