# C++ Robotics Geometry Library

A comprehensive robotics and geometry library built with Eigen, providing mathematical utilities for robotics, mechanics, and 3D geometry applications.

## Overview

This library extends Eigen with high-level geometric primitives and operations commonly used in robotics, computer vision, and mechanical engineering. It provides intuitive APIs for working with 3D poses, velocity vectors (twists), force vectors (wrenches), and coordinate transformations.

## Features

### Core Geometry Types

- **Pose3D**: 3D poses combining position (translation) and orientation (quaternion)
- **Twist**: 6D velocity vectors representing linear and angular velocities
- **Wrench**: 6D force vectors representing forces and torques
- **Transforms**: Coordinate frame transformations and utilities

### Mathematical Utilities

- Angle normalization and conversion (degrees/radians)
- Vector operations (projection, normalization, cross products)
- Rotation utilities (axis-angle, Euler angles, rotation matrices)
- Statistical functions (mean, covariance, PCA)
- Distance metrics (Euclidean, Manhattan, Chebyshev)

### Robotics Applications

- Pose composition and interpolation
- Adjoint transformations for twists and wrenches
- Power calculations (wrench · twist)
- Trajectory generation and analysis
- Force analysis for robotic systems

## Building the Project

### Dependencies

- **Eigen3** (>= 3.4.0) - Linear algebra library
- **CLI11** (>= 2.3.2) - Command-line interface library
- **CMake** (>= 3.20) - Build system
- **C++20 compatible compiler**

### Build Instructions

```bash
# Install dependencies (Ubuntu/Debian)
sudo apt-get update
sudo apt-get install libeigen3-dev libcli11-dev cmake build-essential

# Clone and build
git clone <repository-url>
cd cpp-robotics-geometry
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# Run demos
./apps/CppProjectTemplate --demo basic
./apps/CppProjectTemplate --demo transforms --verbose
./apps/CppProjectTemplate --demo robotics
```

## Usage Examples

### Basic Pose Operations

```cpp
#include "cpp_project_template/geometry.hpp"
using namespace cpp_project_template::geometry;

// Create poses
Pose3D pose1(Eigen::Vector3d(1, 2, 3), Eigen::Quaterniond::Identity());
Pose3D pose2 = Pose3D::AxisAngle(Eigen::Vector3d::UnitZ(), utilities::degreesToRadians(45));

// Compose poses
Pose3D composed = pose1 * pose2;

// Transform points
Eigen::Vector3d point(0, 0, 1);
Eigen::Vector3d transformed_point = pose1 * point;
```

### Twist and Wrench Operations

```cpp
// Create velocity vector
Twist twist(Eigen::Vector3d(1, 0, 0),    // Linear velocity
           Eigen::Vector3d(0, 0, 0.5));  // Angular velocity

// Create force vector
Wrench wrench = Wrench::PointForce(Eigen::Vector3d(10, 0, 0),  // Force
                                  Eigen::Vector3d(0.5, 0, 0),  // Point of application
                                  Eigen::Vector3d::Zero());    // Reference point

// Calculate power
double power = wrench.power(twist);  // P = F·v + τ·ω
```

### Coordinate Transformations

```cpp
// Transform between coordinate frames
Pose3D robot_base = Pose3D::Translation(Eigen::Vector3d(5, 3, 0));
Twist body_twist(Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0, 0, 0.5));

// Convert from body frame to spatial frame
Twist spatial_twist = transforms::transformTwist(body_twist, robot_base);

// Transform multiple points
std::vector<Eigen::Vector3d> points = {/* ... */};
auto transformed_points = transforms::transformPoints(points, robot_base);
```

## Library Structure

```
include/cpp_project_template/geometry/
├── geometry.hpp          # Main header
├── pose3d.hpp           # 3D pose class
├── twist.hpp            # 6D twist (velocity) class
├── wrench.hpp           # 6D wrench (force) class
├── transforms.hpp       # Transformation utilities
└── utilities.hpp        # Mathematical utilities

src/
├── pose3d.cpp
├── twist.cpp
├── wrench.cpp
├── transforms.cpp
└── utilities.cpp
```

## Demo Applications

The library includes comprehensive demos showcasing:

1. **Basic Operations** (`--demo basic`):
   - Pose creation and composition
   - Twist operations and magnitude calculations
   - Wrench operations and power calculations

2. **Transformations** (`--demo transforms`):
   - Coordinate frame transformations
   - Point cloud transformations
   - Adjoint transformations for twists

3. **Robotics Applications** (`--demo robotics`):
   - Trajectory generation and analysis
   - Velocity estimation between waypoints
   - Force analysis with gravity and contact forces

## Mathematical Background

This library implements standard robotics mathematics:

- **SE(3)** poses using quaternions for rotation representation
- **se(3)** twists for velocity representation
- **Adjoint transformations** for coordinate frame changes
- **Screw theory** for unified treatment of motion and force

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Dependencies and Acknowledgments

- [Eigen](https://eigen.tuxfamily.org/) - Excellent linear algebra library
- [CLI11](https://github.com/CLIUtils/CLI11) - Modern command-line parser
- Modern C++ best practices and design patterns

## Version

Current version: 1.0.0

For detailed API documentation, build with Doxygen:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make docs  # Requires Doxygen
```