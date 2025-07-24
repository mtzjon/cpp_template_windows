# Core library
add_library(CppProjectTemplate_Core
    src/utilities.cpp
)

# Create alias for namespaced target
add_library(CppProjectTemplate::Core ALIAS CppProjectTemplate_Core)

# Set library properties
set_target_properties(CppProjectTemplate_Core PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    EXPORT_NAME Core
)

# Target include directories
target_include_directories(CppProjectTemplate_Core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(CppProjectTemplate_Core
    PUBLIC
        Eigen3::Eigen
)

# Compiler features
target_compile_features(CppProjectTemplate_Core
    PUBLIC
        cxx_std_20
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate_Core PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate_Core PRIVATE -Wall -Wextra -Wpedantic)
endif()