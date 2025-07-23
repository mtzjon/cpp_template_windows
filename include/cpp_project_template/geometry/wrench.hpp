#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>

namespace cpp_project_template {
namespace geometry {

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
     * @param fx Force in X direction
     * @param fy Force in Y direction
     * @param fz Force in Z direction
     * @param tx Torque around X axis
     * @param ty Torque around Y axis
     * @param tz Torque around Z axis
     */
    Wrench(double fx, double fy, double fz, double tx, double ty, double tz);

    // Accessors
    const Vector3d& force() const { return force_; }
    const Vector3d& torque() const { return torque_; }
    
    Vector3d& force() { return force_; }
    Vector3d& torque() { return torque_; }

    /**
     * @brief Get the full 6D wrench vector
     * @return 6D vector [force; torque]
     */
    Vector6d vector() const;

    /**
     * @brief Set from 6D vector
     * @param wrench_vector 6D vector [force; torque]
     */
    void setVector(const Vector6d& wrench_vector);

    // Component accessors
    double fx() const { return force_(0); }
    double fy() const { return force_(1); }
    double fz() const { return force_(2); }
    double tx() const { return torque_(0); }
    double ty() const { return torque_(1); }
    double tz() const { return torque_(2); }

    // Operators
    /**
     * @brief Addition of two wrenches
     * @param other Another wrench
     * @return Sum of the two wrenches
     */
    Wrench operator+(const Wrench& other) const;

    /**
     * @brief Subtraction of two wrenches
     * @param other Another wrench
     * @return Difference of the two wrenches
     */
    Wrench operator-(const Wrench& other) const;

    /**
     * @brief Scalar multiplication
     * @param scalar Scalar value
     * @return Scaled wrench
     */
    Wrench operator*(double scalar) const;

    /**
     * @brief Scalar division
     * @param scalar Scalar value
     * @return Scaled wrench
     */
    Wrench operator/(double scalar) const;

    /**
     * @brief Unary minus
     * @return Negated wrench
     */
    Wrench operator-() const;

    /**
     * @brief Addition assignment
     * @param other Another wrench
     * @return Reference to this wrench
     */
    Wrench& operator+=(const Wrench& other);

    /**
     * @brief Subtraction assignment
     * @param other Another wrench
     * @return Reference to this wrench
     */
    Wrench& operator-=(const Wrench& other);

    /**
     * @brief Scalar multiplication assignment
     * @param scalar Scalar value
     * @return Reference to this wrench
     */
    Wrench& operator*=(double scalar);

    /**
     * @brief Scalar division assignment
     * @param scalar Scalar value
     * @return Reference to this wrench
     */
    Wrench& operator/=(double scalar);

    /**
     * @brief Equality comparison with tolerance
     * @param other Wrench to compare with
     * @param tolerance Tolerance for comparison (default: 1e-9)
     * @return True if wrenches are equal within tolerance
     */
    bool isApprox(const Wrench& other, double tolerance = 1e-9) const;

    // Wrench operations
    /**
     * @brief Compute the magnitude (norm) of the wrench
     * @return Euclidean norm of the 6D wrench vector
     */
    double norm() const;

    /**
     * @brief Compute squared magnitude of the wrench
     * @return Squared Euclidean norm of the 6D wrench vector
     */
    double squaredNorm() const;

    /**
     * @brief Normalize the wrench to unit magnitude
     * @return Normalized wrench (unit magnitude)
     */
    Wrench normalized() const;

    /**
     * @brief Normalize this wrench in-place
     */
    void normalize();

    /**
     * @brief Check if wrench is approximately zero
     * @param tolerance Tolerance for zero check (default: 1e-9)
     * @return True if wrench is approximately zero
     */
    bool isZero(double tolerance = 1e-9) const;

    /**
     * @brief Dot product with another wrench
     * @param other Another wrench
     * @return Dot product of the 6D vectors
     */
    double dot(const Wrench& other) const;

    /**
     * @brief Cross product for the force parts
     * @param other Another wrench
     * @return Cross product of force vectors
     */
    Vector3d forceCross(const Wrench& other) const;

    /**
     * @brief Cross product for the torque parts
     * @param other Another wrench
     * @return Cross product of torque vectors
     */
    Vector3d torqueCross(const Wrench& other) const;

    /**
     * @brief Compute power (dot product with twist)
     * @param twist Twist vector representing velocity
     * @return Power (scalar) = wrench · twist
     */
    double power(const class Twist& twist) const;

    // Static factory methods
    /**
     * @brief Create zero wrench
     * @return Zero wrench (all components zero)
     */
    static Wrench Zero();

    /**
     * @brief Create force wrench (force only)
     * @param force 3D force vector
     * @return Wrench with only force component
     */
    static Wrench Force(const Vector3d& force);

    /**
     * @brief Create torque wrench (torque only)
     * @param torque 3D torque vector
     * @return Wrench with only torque component
     */
    static Wrench Torque(const Vector3d& torque);

    /**
     * @brief Create wrench from axis and magnitudes
     * @param axis 3D axis direction (will be normalized)
     * @param force_magnitude Force magnitude along the axis
     * @param torque_magnitude Torque magnitude around the axis
     * @return Wrench along the specified axis
     */
    static Wrench AxisForce(const Vector3d& axis, double force_magnitude, double torque_magnitude);

    /**
     * @brief Create wrench from point force
     * @param force 3D force vector
     * @param point_of_application 3D point where force is applied
     * @param reference_point 3D reference point for torque calculation
     * @return Wrench equivalent to force applied at given point
     */
    static Wrench PointForce(const Vector3d& force, const Vector3d& point_of_application, 
                            const Vector3d& reference_point = Vector3d::Zero());

    // Utility methods
    /**
     * @brief String representation of the wrench
     * @return String containing force and torque information
     */
    std::string toString() const;

    /**
     * @brief Stream output operator
     */
    friend std::ostream& operator<<(std::ostream& os, const Wrench& wrench);

    /**
     * @brief Scalar multiplication (scalar * wrench)
     */
    friend Wrench operator*(double scalar, const Wrench& wrench);

private:
    Vector3d force_;   ///< Force vector [fx, fy, fz]
    Vector3d torque_;  ///< Torque vector [tx, ty, tz]
};

} // namespace geometry
} // namespace cpp_project_template