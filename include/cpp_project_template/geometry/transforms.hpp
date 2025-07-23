#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace cpp_project_template {
namespace geometry {

// Forward declarations
class Pose3D;
class Twist;
class Wrench;

/**
 * @brief Utility functions for geometric transformations
 * 
 * This namespace provides functions for transforming poses, twists, wrenches,
 * and other geometric objects between different coordinate frames.
 */
namespace transforms {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;

// Pose transformations
/**
 * @brief Transform a pose from one frame to another
 * @param pose Pose in the source frame
 * @param transform Transformation from source to target frame
 * @return Pose in the target frame
 */
Pose3D transformPose(const Pose3D& pose, const Pose3D& transform);

/**
 * @brief Transform a pose by a transformation matrix
 * @param pose Pose to transform
 * @param transform 4x4 transformation matrix
 * @return Transformed pose
 */
Pose3D transformPose(const Pose3D& pose, const Matrix4d& transform);

/**
 * @brief Inverse transform a pose
 * @param pose Pose in the target frame
 * @param transform Transformation from source to target frame
 * @return Pose in the source frame
 */
Pose3D inverseTransformPose(const Pose3D& pose, const Pose3D& transform);

// Twist transformations
/**
 * @brief Transform a twist from one frame to another
 * @param twist Twist in the source frame
 * @param transform Transformation from source to target frame
 * @return Twist in the target frame
 */
Twist transformTwist(const Twist& twist, const Pose3D& transform);

/**
 * @brief Get the adjoint matrix for twist transformation
 * @param transform Pose representing the transformation
 * @return 6x6 adjoint matrix for transforming twists
 */
Matrix6d getAdjoint(const Pose3D& transform);

/**
 * @brief Transform a twist using an adjoint matrix
 * @param twist Twist to transform
 * @param adjoint 6x6 adjoint matrix
 * @return Transformed twist
 */
Twist transformTwist(const Twist& twist, const Matrix6d& adjoint);

// Wrench transformations
/**
 * @brief Transform a wrench from one frame to another
 * @param wrench Wrench in the source frame
 * @param transform Transformation from source to target frame
 * @return Wrench in the target frame
 */
Wrench transformWrench(const Wrench& wrench, const Pose3D& transform);

/**
 * @brief Get the adjoint transpose matrix for wrench transformation
 * @param transform Pose representing the transformation
 * @return 6x6 adjoint transpose matrix for transforming wrenches
 */
Matrix6d getAdjointTranspose(const Pose3D& transform);

/**
 * @brief Transform a wrench using an adjoint transpose matrix
 * @param wrench Wrench to transform
 * @param adjoint_transpose 6x6 adjoint transpose matrix
 * @return Transformed wrench
 */
Wrench transformWrench(const Wrench& wrench, const Matrix6d& adjoint_transpose);

// Point and vector transformations
/**
 * @brief Transform a 3D point by a pose
 * @param point Point to transform
 * @param transform Pose representing the transformation
 * @return Transformed point
 */
Vector3d transformPoint(const Vector3d& point, const Pose3D& transform);

/**
 * @brief Transform a direction vector (rotation only)
 * @param vector Direction vector to transform
 * @param transform Pose representing the transformation
 * @return Transformed direction vector
 */
Vector3d transformVector(const Vector3d& vector, const Pose3D& transform);

/**
 * @brief Transform multiple points
 * @param points Vector of points to transform
 * @param transform Pose representing the transformation
 * @return Vector of transformed points
 */
std::vector<Vector3d> transformPoints(const std::vector<Vector3d>& points, const Pose3D& transform);

// Coordinate frame utilities
/**
 * @brief Create a coordinate frame from X and Y axes
 * @param x_axis X-axis direction (will be normalized)
 * @param y_axis Y-axis direction (will be made orthogonal to X and normalized)
 * @return Rotation matrix representing the coordinate frame
 */
Matrix3d createFrame(const Vector3d& x_axis, const Vector3d& y_axis);

/**
 * @brief Create a coordinate frame from Z axis (up direction)
 * @param z_axis Z-axis direction (will be normalized)
 * @param reference_vector Reference vector for determining X and Y axes
 * @return Rotation matrix representing the coordinate frame
 */
Matrix3d createFrameFromZ(const Vector3d& z_axis, const Vector3d& reference_vector = Vector3d::UnitX());

/**
 * @brief Look-at transformation
 * @param from Source position
 * @param to Target position
 * @param up Up direction vector
 * @return Pose that looks from 'from' towards 'to' with given up direction
 */
Pose3D lookAt(const Vector3d& from, const Vector3d& to, const Vector3d& up = Vector3d::UnitZ());

// Chain transformations
/**
 * @brief Compose multiple poses in sequence
 * @param poses Vector of poses to compose (applied left to right)
 * @return Composed pose
 */
Pose3D composePoses(const std::vector<Pose3D>& poses);

/**
 * @brief Get relative pose between two poses
 * @param from Source pose
 * @param to Target pose
 * @return Relative pose from 'from' to 'to'
 */
Pose3D relativePose(const Pose3D& from, const Pose3D& to);

// Conversion utilities
/**
 * @brief Convert rotation matrix to quaternion
 * @param rotation 3x3 rotation matrix
 * @return Unit quaternion
 */
Quaterniond matrixToQuaternion(const Matrix3d& rotation);

/**
 * @brief Convert quaternion to rotation matrix
 * @param quaternion Unit quaternion
 * @return 3x3 rotation matrix
 */
Matrix3d quaternionToMatrix(const Quaterniond& quaternion);

/**
 * @brief Convert pose to transformation matrix
 * @param pose Input pose
 * @return 4x4 homogeneous transformation matrix
 */
Matrix4d poseToMatrix(const Pose3D& pose);

/**
 * @brief Convert transformation matrix to pose
 * @param matrix 4x4 homogeneous transformation matrix
 * @return Pose object
 */
Pose3D matrixToPose(const Matrix4d& matrix);

/**
 * @brief Extract translation from transformation matrix
 * @param matrix 4x4 transformation matrix
 * @return 3D translation vector
 */
Vector3d extractTranslation(const Matrix4d& matrix);

/**
 * @brief Extract rotation from transformation matrix
 * @param matrix 4x4 transformation matrix
 * @return 3x3 rotation matrix
 */
Matrix3d extractRotation(const Matrix4d& matrix);

// Interpolation utilities
/**
 * @brief Spherical linear interpolation between rotations
 * @param q1 Start quaternion
 * @param q2 End quaternion
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated quaternion
 */
Quaterniond slerp(const Quaterniond& q1, const Quaterniond& q2, double t);

/**
 * @brief Interpolate between poses using SLERP for rotation
 * @param pose1 Start pose
 * @param pose2 End pose
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated pose
 */
Pose3D interpolatePoses(const Pose3D& pose1, const Pose3D& pose2, double t);

} // namespace transforms
} // namespace geometry
} // namespace cpp_project_template