#include <catch2/catch_session.hpp>
#include <catch2/catch_test_macros.hpp>

#include "cpp_project_template/logger.hpp"

#include <sstream>
#include <string>
#include <vector>

using namespace cpp_project_template;

// Integration tests that test multiple components together
TEST_CASE("Integration Tests", "[integration]") {
    SECTION("Logger initialization and multiple operations") {
        // Initialize a logger for integration testing
        initializeGlobalLogger("integration_test", Logger::Level::Debug);
        auto& logger = getGlobalLogger();

        // Perform multiple operations
        logger.debug("Starting integration test");
        logger.info("Processing data...");

        // Simulate some data processing
        std::vector<int> data{1, 2, 3, 4, 5};
        int sum = 0;
        for (const auto& value : data) {
            sum += value;
        }

        logger.info("Data processing completed. Sum: {}", sum);
        REQUIRE(sum == 15);

        logger.debug("Integration test completed successfully");
    }

    SECTION("Error handling integration") {
        Logger logger("error_test");

        try {
            // Simulate an error condition
            throw std::runtime_error("Simulated error for testing");
        } catch (const std::exception& e) {
            logger.error("Caught exception: {}", e.what());
            REQUIRE(std::string(e.what()) == "Simulated error for testing");
        }
    }
}

TEST_CASE("Performance Tests", "[performance]") {
    SECTION("Logger performance with many messages") {
        Logger logger("perf_test");
        logger.setLevel(Logger::Level::Info);

        // Log many messages quickly
        const int messageCount = 1000;
        for (int i = 0; i < messageCount; ++i) {
            logger.info("Performance test message {}", i);
        }

        // If we get here without hanging, the test passes
        SUCCEED("Performance test completed");
    }

    SECTION("Logger performance with different levels") {
        Logger logger("level_perf_test");

        // Test all levels
        std::vector<Logger::Level> levels = {Logger::Level::Trace,
                                             Logger::Level::Debug,
                                             Logger::Level::Info,
                                             Logger::Level::Warning,
                                             Logger::Level::Error,
                                             Logger::Level::Critical};

        for (auto level : levels) {
            logger.setLevel(level);
            // Log messages at different levels
            logger.trace("Trace message");
            logger.debug("Debug message");
            logger.info("Info message");
            logger.warning("Warning message");
            logger.error("Error message");
            logger.critical("Critical message");
        }

        SUCCEED("Level performance test completed");
    }
}