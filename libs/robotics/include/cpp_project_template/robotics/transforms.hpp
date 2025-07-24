#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

namespace cpp_project_template {

// Forward declarations
namespace geometry { class Pose3D; }
namespace robotics { class Twist; class Wrench; }

namespace robotics {
namespace transforms {

using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Quaterniond = Eigen::Quaterniond;

// Twist transformations
/**
 * @brief Transform a twist from one frame to another
 * @param twist Twist in the source frame
 * @param transform Transformation from source to target frame
 * @return Twist in the target frame
 */
Twist transformTwist(const Twist& twist, const geometry::Pose3D& transform);

/**
 * @brief Get the adjoint matrix for twist transformation
 * @param transform Pose representing the transformation
 * @return 6x6 adjoint matrix for transforming twists
 */
Matrix6d getAdjoint(const geometry::Pose3D& transform);

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
Wrench transformWrench(const Wrench& wrench, const geometry::Pose3D& transform);

/**
 * @brief Get the adjoint transpose matrix for wrench transformation
 * @param transform Pose representing the transformation
 * @return 6x6 adjoint transpose matrix for transforming wrenches
 */
Matrix6d getAdjointTranspose(const geometry::Pose3D& transform);

/**
 * @brief Transform a wrench using an adjoint transpose matrix
 * @param wrench Wrench to transform
 * @param adjoint_transpose 6x6 adjoint transpose matrix
 * @return Transformed wrench
 */
Wrench transformWrench(const Wrench& wrench, const Matrix6d& adjoint_transpose);

// Velocity and acceleration transformations
/**
 * @brief Transform velocity at a point
 * @param velocity Linear velocity at the point
 * @param point Position of the point
 * @param angular_velocity Angular velocity of the frame
 * @return Velocity in the new frame
 */
Vector3d transformPointVelocity(const Vector3d& velocity, const Vector3d& point, 
                               const Vector3d& angular_velocity);

/**
 * @brief Compute velocity of a point given body twist
 * @param body_twist Twist of the rigid body
 * @param point Position of the point on the body
 * @return Velocity of the point
 */
Vector3d pointVelocity(const Twist& body_twist, const Vector3d& point);

// Screw motion utilities
/**
 * @brief Create a twist from screw axis and velocity
 * @param screw_axis 6D screw axis [axis; moment]
 * @param velocity Velocity along the screw
 * @return Twist representing the screw motion
 */
Twist screwMotion(const Eigen::Matrix<double, 6, 1>& screw_axis, double velocity);

/**
 * @brief Decompose twist into screw axis and velocity
 * @param twist Input twist
 * @param screw_axis Output 6D screw axis
 * @param velocity Output velocity magnitude
 */
void decomposeTwist(const Twist& twist, Eigen::Matrix<double, 6, 1>& screw_axis, double& velocity);

} // namespace transforms
} // namespace robotics
} // namespace cpp_project_template