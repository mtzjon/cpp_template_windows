# Add Catch2 for unit testing
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# Add Google Test for additional testing
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
FetchContent_MakeAvailable(googletest)

# Test executable using Catch2
add_executable(${PROJECT_NAME}_tests_catch2
    test_logger_catch2.cpp
    test_main_catch2.cpp
)

target_link_libraries(${PROJECT_NAME}_tests_catch2
    PRIVATE
        ${PROJECT_NAME}::${PROJECT_NAME}
        Catch2::Catch2WithMain
)

target_compile_features(${PROJECT_NAME}_tests_catch2 PRIVATE cxx_std_20)

# Test executable using Google Test
add_executable(${PROJECT_NAME}_tests_gtest
    test_logger_gtest.cpp
)

target_link_libraries(${PROJECT_NAME}_tests_gtest
    PRIVATE
        ${PROJECT_NAME}::${PROJECT_NAME}
        gtest_main
        gmock_main
)

target_compile_features(${PROJECT_NAME}_tests_gtest PRIVATE cxx_std_20)

# Include Catch2 CMake modules
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(Catch)

# Discover Catch2 tests
catch_discover_tests(${PROJECT_NAME}_tests_catch2)

# Discover Google tests
include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME}_tests_gtest)

# Add custom test target
add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS ${PROJECT_NAME}_tests_catch2 ${PROJECT_NAME}_tests_gtest
    COMMENT "Running all tests"
)

# Compiler-specific options for tests
if(MSVC)
    target_compile_options(${PROJECT_NAME}_tests_catch2 PRIVATE /W4)
    target_compile_options(${PROJECT_NAME}_tests_gtest PRIVATE /W4)
else()
    target_compile_options(${PROJECT_NAME}_tests_catch2 PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(${PROJECT_NAME}_tests_gtest PRIVATE -Wall -Wextra -Wpedantic)
endif()