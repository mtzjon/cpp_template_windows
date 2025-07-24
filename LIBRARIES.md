# Multi-Library Project Structure

This project demonstrates how to create **separated libraries** within a single C++ project, providing better modularity, reusability, and maintainability.

## Library Architecture

The project is structured into three main libraries with clear dependencies:

```
┌─────────────────┐
│    Robotics     │  ← High-level robotics functionality
│    Library      │
└─────────┬───────┘
          │ depends on
          ▼
┌─────────────────┐
│    Geometry     │  ← 3D geometry primitives
│    Library      │
└─────────┬───────┘
          │ depends on
          ▼
┌─────────────────┐
│      Core       │  ← Basic mathematical utilities
│     Library     │
└─────────────────┘
```

## Root `src/` and `include/` Directories

The root `src/` and `include/` directories serve important purposes in a multi-library project:

### **Root `include/` Directory**
Contains **project-wide headers** that:
- **Configuration**: Project-wide compile-time settings (`config.hpp`)
- **Integration APIs**: High-level APIs combining multiple libraries (`integration.hpp`)
- **Umbrella headers**: Convenience headers that include everything (`all.hpp`)
- **Compatibility**: Legacy or compatibility headers for older APIs

### **Root `src/` Directory** 
Contains **integration library implementation** that:
- **Integration Library**: High-level functionality combining the separated libraries
- **Project coordination**: Code that doesn't belong to any specific library
- **Cross-library utilities**: Functions that use multiple libraries together

### 4. Integration Library (`CppProjectTemplate::Integration`)

**Purpose**: Provides high-level APIs that combine functionality from Core, Geometry, and Robotics libraries.

**Location**: `src/` (root directory)

**Headers**:
- `cpp_project_template/integration.hpp` - Main integration header
- `cpp_project_template/config.hpp` - Project configuration

**Key Features**:
- **MotionPlanner**: Trajectory planning combining geometry and robotics
- **ForceAnalyzer**: Force analysis and equilibrium checking
- **geometric_utils**: Advanced path generation (circular, helical)
- **kinematics**: Basic robot kinematics utilities

**Dependencies**: 
- Core Library
- Geometry Library  
- Robotics Library
- Eigen3

**Example Usage**:
```cpp
#include "cpp_project_template/integration.hpp"
using namespace cpp_project_template::integration;

// High-level trajectory planning
auto trajectory = MotionPlanner::planLinearTrajectory(start_pose, end_pose);
auto pose_at_time = trajectory.getPoseAtTime(1.5);

// Advanced path generation
auto circular_path = geometric_utils::generateCircularPath(center, radius);
auto helical_path = geometric_utils::generateHelicalPath(center, radius, pitch);

// Force analysis
bool equilibrium = ForceAnalyzer::isInEquilibrium(force_points);
```

## Library Details

### 1. Core Library (`CppProjectTemplate::Core`)

**Purpose**: Provides fundamental mathematical utilities used across the project.

**Location**: `libs/core/`

**Headers**:
- `cpp_project_template/core.hpp` - Main header
- `cpp_project_template/core/utilities.hpp` - Mathematical utilities

**Key Features**:
- Mathematical constants (PI, angle conversions)
- Angle normalization and utilities
- Vector operations (projection, normalization, angle between vectors)
- Numerical utilities (clamping, interpolation, smooth step functions)
- Distance metrics (Euclidean, Manhattan, Chebyshev)
- Statistical functions (mean, covariance)

**Dependencies**: 
- Eigen3 (for vector/matrix operations)

**Example Usage**:
```cpp
#include "cpp_project_template/core.hpp"
using namespace cpp_project_template::core::utilities;

double angle_rad = degreesToRadians(45.0);
double normalized = normalizeAngle(angle_rad);
double dist = distance(Vector3d(1,2,3), Vector3d(4,5,6));
```

### 2. Geometry Library (`CppProjectTemplate::Geometry`)

**Purpose**: Provides 3D geometry primitives and operations.

**Location**: `libs/geometry/`

**Headers**:
- `cpp_project_template/geometry.hpp` - Main header
- `cpp_project_template/geometry/pose3d.hpp` - 3D pose class

**Key Features**:
- **Pose3D class**: Combines 3D position (Vector3d) and orientation (Quaterniond)
- Pose composition and inverse operations
- Transformation matrix conversions
- Euler angle support
- Pose interpolation (SLERP for rotations)
- Distance calculations between poses

**Dependencies**: 
- Core Library
- Eigen3

**Example Usage**:
```cpp
#include "cpp_project_template/geometry.hpp"
using namespace cpp_project_template::geometry;

Pose3D pose1(Vector3d(1,2,3), Quaterniond::Identity());
Pose3D pose2 = Pose3D::AxisAngle(Vector3d::UnitZ(), M_PI/4);
Pose3D composed = pose1 * pose2;
Vector3d point_transformed = pose1 * Vector3d(0,0,1);
```

### 3. Robotics Library (`CppProjectTemplate::Robotics`)

**Purpose**: Provides robotics-specific data structures and algorithms.

**Location**: `libs/robotics/`

**Headers**:
- `cpp_project_template/robotics.hpp` - Main header
- `cpp_project_template/robotics/twist.hpp` - 6D velocity vectors
- `cpp_project_template/robotics/wrench.hpp` - 6D force vectors
- `cpp_project_template/robotics/transforms.hpp` - Coordinate transformations

**Key Features**:
- **Twist class**: 6D velocity vectors (linear + angular velocity)
- **Wrench class**: 6D force vectors (force + torque)
- **Transforms**: Adjoint transformations for coordinate frame changes
- Power calculations (wrench · twist)
- Screw motion utilities

**Dependencies**: 
- Core Library
- Geometry Library
- Eigen3

**Example Usage**:
```cpp
#include "cpp_project_template/robotics.hpp"
using namespace cpp_project_template::robotics;

Twist twist(Vector3d(1,0,0), Vector3d(0,0,1));
Wrench wrench = Wrench::Force(Vector3d(10,0,0));
double power = wrench.power(twist);

// Transform between coordinate frames
geometry::Pose3D transform(Vector3d(1,2,3), Quaterniond::Identity());
Twist spatial_twist = transforms::transformTwist(twist, transform);
```

## Build System

Each library has its own `CMakeLists.txt` with proper dependency management:

### Library Targets

```cmake
# Individual library targets
CppProjectTemplate::Core         # libCppProjectTemplate_Core.a
CppProjectTemplate::Geometry     # libCppProjectTemplate_Geometry.a  
CppProjectTemplate::Robotics     # libCppProjectTemplate_Robotics.a
CppProjectTemplate::Integration  # libCppProjectTemplate_Integration.a

# Umbrella target (links all libraries)
CppProjectTemplate::CppProjectTemplate
```

### CMake Structure

```
CMakeLists.txt                 # Main project configuration
src/CMakeLists.txt            # Integration library build rules  
libs/
├── core/CMakeLists.txt       # Core library build rules
├── geometry/CMakeLists.txt   # Geometry library build rules
└── robotics/CMakeLists.txt   # Robotics library build rules
```

### Using Individual Libraries

You can link against specific libraries as needed:

```cmake
# Use only core utilities
target_link_libraries(my_target PRIVATE CppProjectTemplate::Core)

# Use geometry primitives (automatically includes Core)
target_link_libraries(my_target PRIVATE CppProjectTemplate::Geometry)

# Use robotics functionality (includes Core + Geometry)
target_link_libraries(my_target PRIVATE CppProjectTemplate::Robotics)

# Use high-level integration APIs (includes all base libraries)
target_link_libraries(my_target PRIVATE CppProjectTemplate::Integration)

# Use everything with umbrella target
target_link_libraries(my_target PRIVATE CppProjectTemplate::CppProjectTemplate)
```

## Benefits of Separated Libraries

### 1. **Modularity**
- Each library has a clear, focused purpose
- Dependencies are explicit and well-defined
- Easy to understand and maintain

### 2. **Reusability**
- Libraries can be used independently
- Core utilities can be shared across projects
- Geometry library useful for graphics/vision applications

### 3. **Build Efficiency**
- Only compile what you need
- Parallel compilation of libraries
- Incremental builds when only one library changes

### 4. **Testing**
- Each library can be tested independently
- Unit tests can focus on specific functionality
- Integration tests verify library interactions

### 5. **Distribution**
- Libraries can be packaged separately
- Users can install only required components
- Reduces binary size for specific use cases

## Usage Examples

### Using All Libraries (Umbrella Header)

```cpp
#include "cpp_project_template/all.hpp"
using namespace cpp_project_template;

// Access all functionality
core::utilities::degreesToRadians(45.0);
geometry::Pose3D pose;
robotics::Twist twist;
```

### Using Individual Libraries

```cpp
// Core only
#include "cpp_project_template/core.hpp"
using namespace cpp_project_template::core::utilities;

// Geometry only (includes Core)
#include "cpp_project_template/geometry.hpp"
using namespace cpp_project_template::geometry;

// Robotics only (includes Core + Geometry)
#include "cpp_project_template/robotics.hpp"
using namespace cpp_project_template::robotics;

// Integration APIs (includes all base libraries)
#include "cpp_project_template/integration.hpp"
using namespace cpp_project_template::integration;

// Project configuration
#include "cpp_project_template/config.hpp"
using namespace cpp_project_template::config;
```

## Demo Application

The included demo application showcases all libraries:

```bash
# Show all library functionality
./build/apps/CppProjectTemplate --demo all

# Test individual libraries
./build/apps/CppProjectTemplate --demo core
./build/apps/CppProjectTemplate --demo geometry
./build/apps/CppProjectTemplate --demo robotics

# Show version information
./build/apps/CppProjectTemplate --versions

# Verbose output with dependency information
./build/apps/CppProjectTemplate --verbose
```

## File Structure

```
# Root directories for project-wide functionality
include/cpp_project_template/
├── config.hpp           # Project configuration
├── integration.hpp      # High-level integration APIs  
└── all.hpp             # Umbrella header including everything

src/
├── CMakeLists.txt      # Integration library build rules
└── integration.cpp     # Integration library implementation

# Separated libraries
libs/
├── core/
│   ├── CMakeLists.txt
│   ├── include/cpp_project_template/
│   │   ├── core.hpp
│   │   └── core/utilities.hpp
│   └── src/utilities.cpp
├── geometry/
│   ├── CMakeLists.txt
│   ├── include/cpp_project_template/
│   │   ├── geometry.hpp
│   │   └── geometry/pose3d.hpp
│   └── src/pose3d.cpp
└── robotics/
    ├── CMakeLists.txt
    ├── include/cpp_project_template/
    │   ├── robotics.hpp
    │   └── robotics/
    │       ├── twist.hpp
    │       ├── wrench.hpp
    │       └── transforms.hpp
    └── src/
        ├── twist.cpp
        ├── wrench.cpp
        └── transforms.cpp
```

## Adding New Libraries

To add a new library to the project:

1. **Create directory structure**:
   ```bash
   mkdir -p libs/mylib/{include/cpp_project_template/mylib,src}
   ```

2. **Create CMakeLists.txt**:
   ```cmake
   add_library(CppProjectTemplate_MyLib src/mylib.cpp)
   add_library(CppProjectTemplate::MyLib ALIAS CppProjectTemplate_MyLib)
   target_link_libraries(CppProjectTemplate_MyLib PUBLIC CppProjectTemplate::Core)
   ```

3. **Add to main CMakeLists.txt**:
   ```cmake
   add_subdirectory(libs/mylib)
   target_link_libraries(CppProjectTemplate INTERFACE CppProjectTemplate::MyLib)
   ```

4. **Update umbrella header**:
   ```cpp
   #include "cpp_project_template/mylib.hpp"
   ```

This modular approach provides a solid foundation for scalable C++ projects with clear separation of concerns and explicit dependency management.