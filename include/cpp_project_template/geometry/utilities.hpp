#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

namespace cpp_project_template {
namespace geometry {

/**
 * @brief Mathematical utilities for robotics and geometry computations
 * 
 * This namespace provides common mathematical functions, constants,
 * and algorithms used in robotics, mechanics, and geometry.
 */
namespace utilities {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Quaterniond = Eigen::Quaterniond;
using AngleAxisd = Eigen::AngleAxisd;

// Mathematical constants
constexpr double PI = M_PI;
constexpr double TWO_PI = 2.0 * M_PI;
constexpr double HALF_PI = M_PI / 2.0;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// Angle utilities
/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline double degreesToRadians(double degrees) {
    return degrees * DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline double radiansToDegrees(double radians) {
    return radians * RAD_TO_DEG;
}

/**
 * @brief Normalize angle to [-π, π]
 * @param angle Angle in radians
 * @return Normalized angle in [-π, π]
 */
double normalizeAngle(double angle);

/**
 * @brief Normalize angle to [0, 2π]
 * @param angle Angle in radians
 * @return Normalized angle in [0, 2π]
 */
double normalizeAnglePositive(double angle);

/**
 * @brief Shortest angular distance between two angles
 * @param from Start angle in radians
 * @param to End angle in radians
 * @return Shortest angular distance in radians
 */
double angularDistance(double from, double to);

// Vector utilities
/**
 * @brief Check if vector is approximately zero
 * @param vector Input vector
 * @param tolerance Tolerance for zero check (default: 1e-9)
 * @return True if vector is approximately zero
 */
bool isZero(const Vector3d& vector, double tolerance = 1e-9);

/**
 * @brief Safe vector normalization (returns zero if input is zero)
 * @param vector Input vector
 * @param tolerance Tolerance for zero check (default: 1e-9)
 * @return Normalized vector or zero vector if input is too small
 */
Vector3d safeNormalize(const Vector3d& vector, double tolerance = 1e-9);

/**
 * @brief Compute angle between two vectors
 * @param v1 First vector
 * @param v2 Second vector
 * @return Angle between vectors in radians [0, π]
 */
double angleBetween(const Vector3d& v1, const Vector3d& v2);

/**
 * @brief Project vector onto another vector
 * @param vector Vector to project
 * @param onto Vector to project onto
 * @return Projected vector
 */
Vector3d projectOnto(const Vector3d& vector, const Vector3d& onto);

/**
 * @brief Compute component of vector perpendicular to another vector
 * @param vector Input vector
 * @param direction Reference direction
 * @return Perpendicular component
 */
Vector3d perpendicularComponent(const Vector3d& vector, const Vector3d& direction);

/**
 * @brief Create orthonormal basis from one vector
 * @param z_axis Primary axis (will be normalized)
 * @return 3x3 matrix with orthonormal columns [x, y, z]
 */
Matrix3d createOrthonormalBasis(const Vector3d& z_axis);

/**
 * @brief Gram-Schmidt orthonormalization of three vectors
 * @param v1 First vector
 * @param v2 Second vector  
 * @param v3 Third vector
 * @return 3x3 matrix with orthonormal columns
 */
Matrix3d gramSchmidt(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3);

// Rotation utilities
/**
 * @brief Create rotation matrix from axis and angle
 * @param axis Rotation axis (will be normalized)
 * @param angle Rotation angle in radians
 * @return 3x3 rotation matrix
 */
Matrix3d axisAngleToMatrix(const Vector3d& axis, double angle);

/**
 * @brief Extract axis and angle from rotation matrix
 * @param rotation 3x3 rotation matrix
 * @param axis Output rotation axis
 * @param angle Output rotation angle in radians
 */
void matrixToAxisAngle(const Matrix3d& rotation, Vector3d& axis, double& angle);

/**
 * @brief Convert Euler angles to rotation matrix (ZYX convention)
 * @param roll Roll angle (X-axis) in radians
 * @param pitch Pitch angle (Y-axis) in radians
 * @param yaw Yaw angle (Z-axis) in radians
 * @return 3x3 rotation matrix
 */
Matrix3d eulerToMatrix(double roll, double pitch, double yaw);

/**
 * @brief Convert rotation matrix to Euler angles (ZYX convention)
 * @param rotation 3x3 rotation matrix
 * @return Vector3d containing [roll, pitch, yaw] in radians
 */
Vector3d matrixToEuler(const Matrix3d& rotation);

/**
 * @brief Create rotation matrix to align one vector with another
 * @param from Source vector
 * @param to Target vector
 * @return Rotation matrix that rotates 'from' to align with 'to'
 */
Matrix3d rotationBetween(const Vector3d& from, const Vector3d& to);

/**
 * @brief Logarithmic map of rotation (rotation vector)
 * @param rotation 3x3 rotation matrix
 * @return 3D rotation vector (axis * angle)
 */
Vector3d rotationToVector(const Matrix3d& rotation);

/**
 * @brief Exponential map of rotation vector
 * @param rotation_vector 3D rotation vector (axis * angle)
 * @return 3x3 rotation matrix
 */
Matrix3d vectorToRotation(const Vector3d& rotation_vector);

// Skew-symmetric matrix utilities
/**
 * @brief Create skew-symmetric matrix from 3D vector
 * @param vector Input 3D vector
 * @return 3x3 skew-symmetric matrix
 */
Matrix3d skewSymmetric(const Vector3d& vector);

/**
 * @brief Extract vector from skew-symmetric matrix
 * @param skew_matrix 3x3 skew-symmetric matrix
 * @return 3D vector
 */
Vector3d unskewSymmetric(const Matrix3d& skew_matrix);

// Numerical utilities
/**
 * @brief Check if two values are approximately equal
 * @param a First value
 * @param b Second value
 * @param tolerance Tolerance for comparison (default: 1e-9)
 * @return True if values are approximately equal
 */
inline bool isApprox(double a, double b, double tolerance = 1e-9) {
    return std::abs(a - b) <= tolerance;
}

/**
 * @brief Clamp value to range [min, max]
 * @param value Input value
 * @param min Minimum value
 * @param max Maximum value
 * @return Clamped value
 */
template<typename T>
inline T clamp(T value, T min, T max) {
    return std::max(min, std::min(max, value));
}

/**
 * @brief Linear interpolation
 * @param a Start value
 * @param b End value
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated value
 */
template<typename T>
inline T lerp(const T& a, const T& b, double t) {
    return a + t * (b - a);
}

/**
 * @brief Smooth step function (3t² - 2t³)
 * @param t Input parameter [0, 1]
 * @return Smooth step value [0, 1]
 */
inline double smoothStep(double t) {
    t = clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

/**
 * @brief Smoother step function (6t⁵ - 15t⁴ + 10t³)
 * @param t Input parameter [0, 1]
 * @return Smoother step value [0, 1]
 */
inline double smootherStep(double t) {
    t = clamp(t, 0.0, 1.0);
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// Statistics utilities
/**
 * @brief Compute mean of vector of 3D points
 * @param points Vector of 3D points
 * @return Mean point
 */
Vector3d mean(const std::vector<Vector3d>& points);

/**
 * @brief Compute covariance matrix of 3D points
 * @param points Vector of 3D points
 * @param mean_point Mean of the points (computed if not provided)
 * @return 3x3 covariance matrix
 */
Matrix3d covariance(const std::vector<Vector3d>& points, 
                   const Vector3d& mean_point = Vector3d::Zero());

/**
 * @brief Principal Component Analysis of 3D points
 * @param points Vector of 3D points
 * @param principal_axes Output: 3x3 matrix with principal axes as columns
 * @param eigenvalues Output: eigenvalues in descending order
 * @return Mean of the points
 */
Vector3d principalComponentAnalysis(const std::vector<Vector3d>& points,
                                   Matrix3d& principal_axes,
                                   Vector3d& eigenvalues);

// Distance metrics
/**
 * @brief Euclidean distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Euclidean distance
 */
inline double distance(const Vector3d& p1, const Vector3d& p2) {
    return (p1 - p2).norm();
}

/**
 * @brief Squared Euclidean distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Squared Euclidean distance
 */
inline double squaredDistance(const Vector3d& p1, const Vector3d& p2) {
    return (p1 - p2).squaredNorm();
}

/**
 * @brief Manhattan distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Manhattan distance (L1 norm)
 */
double manhattanDistance(const Vector3d& p1, const Vector3d& p2);

/**
 * @brief Chebyshev distance between two points
 * @param p1 First point
 * @param p2 Second point
 * @return Chebyshev distance (L∞ norm)
 */
double chebyshevDistance(const Vector3d& p1, const Vector3d& p2);

} // namespace utilities
} // namespace geometry
} // namespace cpp_project_template