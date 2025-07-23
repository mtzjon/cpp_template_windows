#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace cpp_project_template {
namespace geometry {

/**
 * @brief Represents a 6D twist vector for rigid body motion
 * 
 * A twist vector combines:
 * - Linear velocity: 3D vector representing translational velocity
 * - Angular velocity: 3D vector representing rotational velocity
 * 
 * This is commonly used in robotics for representing instantaneous
 * rigid body motion, joint velocities, and differential kinematics.
 */
class Twist {
public:
    using Vector3d = Eigen::Vector3d;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    /**
     * @brief Default constructor - creates zero twist
     */
    Twist();

    /**
     * @brief Constructor from linear and angular velocity vectors
     * @param linear 3D linear velocity vector
     * @param angular 3D angular velocity vector
     */
    Twist(const Vector3d& linear, const Vector3d& angular);

    /**
     * @brief Constructor from 6D vector [linear; angular]
     * @param twist_vector 6D vector containing [vx, vy, vz, wx, wy, wz]
     */
    explicit Twist(const Vector6d& twist_vector);

    /**
     * @brief Constructor from individual components
     * @param vx Linear velocity in X direction
     * @param vy Linear velocity in Y direction
     * @param vz Linear velocity in Z direction
     * @param wx Angular velocity around X axis
     * @param wy Angular velocity around Y axis
     * @param wz Angular velocity around Z axis
     */
    Twist(double vx, double vy, double vz, double wx, double wy, double wz);

    // Accessors
    const Vector3d& linear() const { return linear_; }
    const Vector3d& angular() const { return angular_; }
    
    Vector3d& linear() { return linear_; }
    Vector3d& angular() { return angular_; }

    /**
     * @brief Get the full 6D twist vector
     * @return 6D vector [linear; angular]
     */
    Vector6d vector() const;

    /**
     * @brief Set from 6D vector
     * @param twist_vector 6D vector [linear; angular]
     */
    void setVector(const Vector6d& twist_vector);

    // Component accessors
    double vx() const { return linear_(0); }
    double vy() const { return linear_(1); }
    double vz() const { return linear_(2); }
    double wx() const { return angular_(0); }
    double wy() const { return angular_(1); }
    double wz() const { return angular_(2); }

    // Operators
    /**
     * @brief Addition of two twists
     * @param other Another twist
     * @return Sum of the two twists
     */
    Twist operator+(const Twist& other) const;

    /**
     * @brief Subtraction of two twists
     * @param other Another twist
     * @return Difference of the two twists
     */
    Twist operator-(const Twist& other) const;

    /**
     * @brief Scalar multiplication
     * @param scalar Scalar value
     * @return Scaled twist
     */
    Twist operator*(double scalar) const;

    /**
     * @brief Scalar division
     * @param scalar Scalar value
     * @return Scaled twist
     */
    Twist operator/(double scalar) const;

    /**
     * @brief Unary minus
     * @return Negated twist
     */
    Twist operator-() const;

    /**
     * @brief Addition assignment
     * @param other Another twist
     * @return Reference to this twist
     */
    Twist& operator+=(const Twist& other);

    /**
     * @brief Subtraction assignment
     * @param other Another twist
     * @return Reference to this twist
     */
    Twist& operator-=(const Twist& other);

    /**
     * @brief Scalar multiplication assignment
     * @param scalar Scalar value
     * @return Reference to this twist
     */
    Twist& operator*=(double scalar);

    /**
     * @brief Scalar division assignment
     * @param scalar Scalar value
     * @return Reference to this twist
     */
    Twist& operator/=(double scalar);

    /**
     * @brief Equality comparison with tolerance
     * @param other Twist to compare with
     * @param tolerance Tolerance for comparison (default: 1e-9)
     * @return True if twists are equal within tolerance
     */
    bool isApprox(const Twist& other, double tolerance = 1e-9) const;

    // Twist operations
    /**
     * @brief Compute the magnitude (norm) of the twist
     * @return Euclidean norm of the 6D twist vector
     */
    double norm() const;

    /**
     * @brief Compute squared magnitude of the twist
     * @return Squared Euclidean norm of the 6D twist vector
     */
    double squaredNorm() const;

    /**
     * @brief Normalize the twist to unit magnitude
     * @return Normalized twist (unit magnitude)
     */
    Twist normalized() const;

    /**
     * @brief Normalize this twist in-place
     */
    void normalize();

    /**
     * @brief Check if twist is approximately zero
     * @param tolerance Tolerance for zero check (default: 1e-9)
     * @return True if twist is approximately zero
     */
    bool isZero(double tolerance = 1e-9) const;

    /**
     * @brief Dot product with another twist
     * @param other Another twist
     * @return Dot product of the 6D vectors
     */
    double dot(const Twist& other) const;

    /**
     * @brief Cross product for the angular parts
     * @param other Another twist
     * @return Cross product of angular velocity vectors
     */
    Vector3d angularCross(const Twist& other) const;

    // Static factory methods
    /**
     * @brief Create zero twist
     * @return Zero twist (all components zero)
     */
    static Twist Zero();

    /**
     * @brief Create linear twist (translation only)
     * @param linear 3D linear velocity vector
     * @return Twist with only linear component
     */
    static Twist Linear(const Vector3d& linear);

    /**
     * @brief Create angular twist (rotation only)
     * @param angular 3D angular velocity vector
     * @return Twist with only angular component
     */
    static Twist Angular(const Vector3d& angular);

    /**
     * @brief Create twist from axis and velocities
     * @param axis 3D axis direction (will be normalized)
     * @param linear_velocity Linear velocity along the axis
     * @param angular_velocity Angular velocity around the axis
     * @return Twist along the specified axis
     */
    static Twist AxisVelocity(const Vector3d& axis, double linear_velocity, double angular_velocity);

    // Utility methods
    /**
     * @brief String representation of the twist
     * @return String containing linear and angular velocity information
     */
    std::string toString() const;

    /**
     * @brief Stream output operator
     */
    friend std::ostream& operator<<(std::ostream& os, const Twist& twist);

    /**
     * @brief Scalar multiplication (scalar * twist)
     */
    friend Twist operator*(double scalar, const Twist& twist);

private:
    Vector3d linear_;   ///< Linear velocity vector [vx, vy, vz]
    Vector3d angular_;  ///< Angular velocity vector [wx, wy, wz]
};

} // namespace geometry
} // namespace cpp_project_template