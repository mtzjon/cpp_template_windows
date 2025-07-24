#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace cpp_project_template {
namespace robotics {

// Forward declaration
class Twist;

/**
 * @brief Represents a 6D wrench vector for rigid body forces
 * 
 * A wrench vector combines:
 * - Force: 3D vector representing translational force
 * - Torque: 3D vector representing rotational torque/moment
 * 
 * This is commonly used in robotics and mechanics for representing
 * forces and torques acting on rigid bodies, contact forces, and
 * actuator outputs.
 */
class Wrench {
public:
    using Vector3d = Eigen::Vector3d;
    using Vector6d = Eigen::Matrix<double, 6, 1>;

    /**
     * @brief Default constructor - creates zero wrench
     */
    Wrench();

    /**
     * @brief Constructor from force and torque vectors
     * @param force 3D force vector
     * @param torque 3D torque vector
     */
    Wrench(const Vector3d& force, const Vector3d& torque);

    /**
     * @brief Constructor from 6D vector [force; torque]
     * @param wrench_vector 6D vector containing [fx, fy, fz, tx, ty, tz]
     */
    explicit Wrench(const Vector6d& wrench_vector);

    /**
     * @brief Constructor from individual components
     */
    Wrench(double fx, double fy, double fz, double tx, double ty, double tz);

    // Accessors
    const Vector3d& force() const { return force_; }
    const Vector3d& torque() const { return torque_; }
    Vector3d& force() { return force_; }
    Vector3d& torque() { return torque_; }
    Vector6d vector() const;
    void setVector(const Vector6d& wrench_vector);

    // Component accessors
    double fx() const { return force_(0); }
    double fy() const { return force_(1); }
    double fz() const { return force_(2); }
    double tx() const { return torque_(0); }
    double ty() const { return torque_(1); }
    double tz() const { return torque_(2); }

    // Operators
    Wrench operator+(const Wrench& other) const;
    Wrench operator-(const Wrench& other) const;
    Wrench operator*(double scalar) const;
    Wrench operator/(double scalar) const;
    Wrench operator-() const;
    Wrench& operator+=(const Wrench& other);
    Wrench& operator-=(const Wrench& other);
    Wrench& operator*=(double scalar);
    Wrench& operator/=(double scalar);
    bool isApprox(const Wrench& other, double tolerance = 1e-9) const;

    // Wrench operations
    double norm() const;
    double squaredNorm() const;
    Wrench normalized() const;
    void normalize();
    bool isZero(double tolerance = 1e-9) const;
    double dot(const Wrench& other) const;
    Vector3d forceCross(const Wrench& other) const;
    Vector3d torqueCross(const Wrench& other) const;
    double power(const Twist& twist) const;

    // Static factory methods
    static Wrench Zero();
    static Wrench Force(const Vector3d& force);
    static Wrench Torque(const Vector3d& torque);
    static Wrench AxisForce(const Vector3d& axis, double force_magnitude, double torque_magnitude);
    static Wrench PointForce(const Vector3d& force, const Vector3d& point_of_application, 
                            const Vector3d& reference_point = Vector3d::Zero());

    // Utility methods
    std::string toString() const;
    friend std::ostream& operator<<(std::ostream& os, const Wrench& wrench);
    friend Wrench operator*(double scalar, const Wrench& wrench);

private:
    Vector3d force_;   ///< Force vector [fx, fy, fz]
    Vector3d torque_;  ///< Torque vector [tx, ty, tz]
};

} // namespace robotics
} // namespace cpp_project_template