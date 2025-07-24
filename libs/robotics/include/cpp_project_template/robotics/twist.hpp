#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace cpp_project_template {
namespace robotics {

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
    Twist operator+(const Twist& other) const;
    Twist operator-(const Twist& other) const;
    Twist operator*(double scalar) const;
    Twist operator/(double scalar) const;
    Twist operator-() const;
    Twist& operator+=(const Twist& other);
    Twist& operator-=(const Twist& other);
    Twist& operator*=(double scalar);
    Twist& operator/=(double scalar);
    bool isApprox(const Twist& other, double tolerance = 1e-9) const;

    // Twist operations
    double norm() const;
    double squaredNorm() const;
    Twist normalized() const;
    void normalize();
    bool isZero(double tolerance = 1e-9) const;
    double dot(const Twist& other) const;
    Vector3d angularCross(const Twist& other) const;

    // Static factory methods
    static Twist Zero();
    static Twist Linear(const Vector3d& linear);
    static Twist Angular(const Vector3d& angular);
    static Twist AxisVelocity(const Vector3d& axis, double linear_velocity, double angular_velocity);

    // Utility methods
    std::string toString() const;
    friend std::ostream& operator<<(std::ostream& os, const Twist& twist);
    friend Twist operator*(double scalar, const Twist& twist);

private:
    Vector3d linear_;   ///< Linear velocity vector [vx, vy, vz]
    Vector3d angular_;  ///< Angular velocity vector [wx, wy, wz]
};

} // namespace robotics
} // namespace cpp_project_template