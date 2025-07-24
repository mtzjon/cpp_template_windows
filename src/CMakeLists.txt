# Root src/ directory - Project Integration Library
# 
# This library provides high-level integration APIs that combine
# functionality from the separated libraries (Core, Geometry, Robotics)

add_library(CppProjectTemplate_Integration
    integration.cpp
)

# Create alias for namespaced target
add_library(CppProjectTemplate::Integration ALIAS CppProjectTemplate_Integration)

# Set library properties
set_target_properties(CppProjectTemplate_Integration PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME Integration
)

# Target include directories
target_include_directories(CppProjectTemplate_Integration
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies - depends on all separated libraries
target_link_libraries(CppProjectTemplate_Integration
    PUBLIC
        CppProjectTemplate::Core
        CppProjectTemplate::Geometry
        CppProjectTemplate::Robotics
        Eigen3::Eigen
)

# Compiler features
target_compile_features(CppProjectTemplate_Integration
    PUBLIC
        cxx_std_20
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate_Integration PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate_Integration PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Optional: Enable specific features for integration library
target_compile_definitions(CppProjectTemplate_Integration
    PRIVATE
        CPP_PROJECT_TEMPLATE_ENABLE_INTEGRATION=1
)
