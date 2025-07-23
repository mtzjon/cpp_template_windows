# Create the main library
add_library(CppProjectTemplate 
    pose3d.cpp
    twist.cpp
    wrench.cpp
    transforms.cpp
    utilities.cpp
)

# Create alias for namespaced target
add_library(CppProjectTemplate::CppProjectTemplate ALIAS CppProjectTemplate)

# Set library properties
set_target_properties(CppProjectTemplate PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME CppProjectTemplate
)

# Target include directories
target_include_directories(CppProjectTemplate
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(CppProjectTemplate
    PUBLIC
        Eigen3::Eigen
)

# Compiler features
target_compile_features(CppProjectTemplate
    PUBLIC
        cxx_std_20
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate PRIVATE -Wall -Wextra -Wpedantic)
endif()
