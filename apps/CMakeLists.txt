# Main application
add_executable(${PROJECT_NAME}_app main.cpp)

# Set target properties
set_target_properties(${PROJECT_NAME}_app PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    OUTPUT_NAME ${PROJECT_NAME}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}_app
    PRIVATE
        ${PROJECT_NAME}
        CLI11::CLI11
        fmt::fmt
        spdlog::spdlog
)

# Compiler-specific options
if(MSVC)
    target_compile_options(${PROJECT_NAME}_app PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME}_app PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Installation is handled in main CMakeLists.txt