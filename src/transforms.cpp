#include "cpp_project_template/geometry/transforms.hpp"
#include "cpp_project_template/geometry/pose3d.hpp"
#include "cpp_project_template/geometry/twist.hpp"
#include "cpp_project_template/geometry/wrench.hpp"

namespace cpp_project_template {
namespace geometry {
namespace transforms {

// Pose transformations
Pose3D transformPose(const Pose3D& pose, const Pose3D& transform) {
    return transform * pose;
}

Pose3D transformPose(const Pose3D& pose, const Matrix4d& transform) {
    return Pose3D(transform) * pose;
}

Pose3D inverseTransformPose(const Pose3D& pose, const Pose3D& transform) {
    return transform.inverse() * pose;
}

// Twist transformations
Twist transformTwist(const Twist& twist, const Pose3D& transform) {
    Matrix6d adj = getAdjoint(transform);
    return transformTwist(twist, adj);
}

Matrix6d getAdjoint(const Pose3D& transform) {
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
Wrench transformWrench(const Wrench& wrench, const Pose3D& transform) {
    Matrix6d adj_T = getAdjointTranspose(transform);
    return transformWrench(wrench, adj_T);
}

Matrix6d getAdjointTranspose(const Pose3D& transform) {
    return getAdjoint(transform).transpose();
}

Wrench transformWrench(const Wrench& wrench, const Matrix6d& adjoint_transpose) {
    Wrench::Vector6d transformed = adjoint_transpose * wrench.vector();
    return Wrench(transformed);
}

// Point and vector transformations
Vector3d transformPoint(const Vector3d& point, const Pose3D& transform) {
    return transform * point;
}

Vector3d transformVector(const Vector3d& vector, const Pose3D& transform) {
    return transform.rotationMatrix() * vector;
}

std::vector<Vector3d> transformPoints(const std::vector<Vector3d>& points, const Pose3D& transform) {
    std::vector<Vector3d> result;
    result.reserve(points.size());
    for (const auto& point : points) {
        result.push_back(transformPoint(point, transform));
    }
    return result;
}

// Basic implementations for other functions
Matrix3d createFrame(const Vector3d& x_axis, const Vector3d& y_axis) {
    Vector3d x = x_axis.normalized();
    Vector3d z = x.cross(y_axis).normalized();
    Vector3d y = z.cross(x);
    
    Matrix3d frame;
    frame.col(0) = x;
    frame.col(1) = y;
    frame.col(2) = z;
    return frame;
}

Pose3D lookAt(const Vector3d& from, const Vector3d& to, const Vector3d& up) {
    Vector3d forward = (to - from).normalized();
    Vector3d right = forward.cross(up).normalized();
    Vector3d actual_up = right.cross(forward);
    
    Matrix3d rotation;
    rotation.col(0) = right;
    rotation.col(1) = actual_up;
    rotation.col(2) = -forward;  // Negative for right-handed coordinate system
    
    return Pose3D(from, Quaterniond(rotation));
}

Pose3D relativePose(const Pose3D& from, const Pose3D& to) {
    return from.inverse() * to;
}

// Conversion utilities
Matrix4d poseToMatrix(const Pose3D& pose) {
    return pose.matrix();
}

Pose3D matrixToPose(const Matrix4d& matrix) {
    return Pose3D(matrix);
}

Vector3d extractTranslation(const Matrix4d& matrix) {
    return matrix.block<3, 1>(0, 3);
}

Matrix3d extractRotation(const Matrix4d& matrix) {
    return matrix.block<3, 3>(0, 0);
}

} // namespace transforms
} // namespace geometry
} // namespace cpp_project_template