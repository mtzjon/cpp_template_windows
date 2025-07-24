# Robotics library
add_library(CppProjectTemplate_Robotics
    src/twist.cpp
    src/wrench.cpp
    src/transforms.cpp
)

# Create alias for namespaced target
add_library(CppProjectTemplate::Robotics ALIAS CppProjectTemplate_Robotics)

# Set library properties
set_target_properties(CppProjectTemplate_Robotics PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME Robotics
)

# Target include directories
target_include_directories(CppProjectTemplate_Robotics
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(CppProjectTemplate_Robotics
    PUBLIC
        Eigen3::Eigen
        CppProjectTemplate::Core
        CppProjectTemplate::Geometry
)

# Compiler features
target_compile_features(CppProjectTemplate_Robotics
    PUBLIC
        cxx_std_20
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate_Robotics PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate_Robotics PRIVATE -Wall -Wextra -Wpedantic)
endif()