# Create the main executable
add_executable(CppProjectTemplate_app main.cpp)

# Set executable properties
set_target_properties(CppProjectTemplate_app PROPERTIES
    OUTPUT_NAME CppProjectTemplate
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

# Link libraries
target_link_libraries(CppProjectTemplate_app
    PRIVATE
        CppProjectTemplate::CppProjectTemplate
        CLI11::CLI11
)

# Compiler options
if(MSVC)
    target_compile_options(CppProjectTemplate_app PRIVATE /W4)
else()
    target_compile_options(CppProjectTemplate_app PRIVATE -Wall -Wextra -Wpedantic)
endif()