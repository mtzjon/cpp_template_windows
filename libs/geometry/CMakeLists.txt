# Geometry library
add_library(CppProjectTemplate_Geometry
    src/pose3d.cpp
)

# Create alias for namespaced target
add_library(CppProjectTemplate::Geometry ALIAS CppProjectTemplate_Geometry)

# Set library properties
set_target_properties(CppProjectTemplate_Geometry PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME Geometry
)

# Target include directories
target_include_directories(CppProjectTemplate_Geometry
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(CppProjectTemplate_Geometry
    PUBLIC
        Eigen3::Eigen
        CppProjectTemplate::Core
)

# Compiler features
target_compile_features(CppProjectTemplate_Geometry
    PUBLIC
        cxx_std_20
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate_Geometry PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate_Geometry PRIVATE -Wall -Wextra -Wpedantic)
endif()