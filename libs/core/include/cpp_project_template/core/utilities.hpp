#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace cpp_project_template {
namespace core {

/**
 * @brief Mathematical utilities and constants
 * 
 * This namespace provides common mathematical functions, constants,
 * and algorithms used across the project.
 */
namespace utilities {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;

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

} // namespace utilities
} // namespace core
} // namespace cpp_project_template