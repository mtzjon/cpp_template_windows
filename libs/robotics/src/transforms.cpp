#include "cpp_project_template/robotics/transforms.hpp"
#include "cpp_project_template/geometry/pose3d.hpp"
#include "cpp_project_template/robotics/twist.hpp"
#include "cpp_project_template/robotics/wrench.hpp"

namespace cpp_project_template {
namespace robotics {
namespace transforms {

// Pose transformations
// Note: Pose transformations moved to geometry library
// Here we focus on robotics-specific transformations

// Twist transformations
Twist transformTwist(const Twist& twist, const geometry::Pose3D& transform) {
    Matrix6d adj = getAdjoint(transform);
    return transformTwist(twist, adj);
}

Matrix6d getAdjoint(const geometry::Pose3D& transform) {
    Matrix6d adj = Matrix6d::Zero();
    Matrix3d R = transform.rotationMatrix();
    Vector3d p = transform.position();
    
    // Skew-symmetric matrix of position
    Matrix3d p_skew;
    p_skew << 0, -p(2), p(1),
              p(2), 0, -p(0),
              -p(1), p(0), 0;
    
    adj.block<3, 3>(0, 0) = R;
    adj.block<3, 3>(0, 3) = p_skew * R;
    adj.block<3, 3>(3, 3) = R;
    
    return adj;
}

Twist transformTwist(const Twist& twist, const Matrix6d& adjoint) {
    Twist::Vector6d transformed = adjoint * twist.vector();
    return Twist(transformed);
}

// Wrench transformations  
Wrench transformWrench(const Wrench& wrench, const geometry::Pose3D& transform) {
    Matrix6d adj_T = getAdjointTranspose(transform);
    return transformWrench(wrench, adj_T);
}

Matrix6d getAdjointTranspose(const geometry::Pose3D& transform) {
    return getAdjoint(transform).transpose();
}

Wrench transformWrench(const Wrench& wrench, const Matrix6d& adjoint_transpose) {
    Wrench::Vector6d transformed = adjoint_transpose * wrench.vector();
    return Wrench(transformed);
}

// Additional robotics-specific transform functions can be added here
// Point and vector transformations are handled by the geometry library

} // namespace transforms
} // namespace robotics
} // namespace cpp_project_template