#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "cpp_project_template/logger.hpp"

#include <memory>
#include <stdexcept>
#include <string>

using namespace cpp_project_template;
using ::testing::_;
using ::testing::Return;
using ::testing::Throw;

class LoggerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Setup for each test case
    }
    
    void TearDown() override {
        // Cleanup after each test case
    }
};

// Parameterized test for different log levels
class LoggerLevelTest : public ::testing::TestWithParam<Logger::Level> {
  protected:
    Logger logger_{"param_test"};
};

TEST_F(LoggerTest, ConstructorDoesNotThrow) {
    EXPECT_NO_THROW(Logger("test_logger"));
}

TEST_F(LoggerTest, LoggerIsMovable) {
    Logger logger1("test_logger");
    Logger logger2 = std::move(logger1);
    EXPECT_NO_THROW(logger2.info("Test message"));
}

TEST_F(LoggerTest, SetLevelDoesNotThrow) {
    Logger logger("test_logger");
    EXPECT_NO_THROW(logger.setLevel(Logger::Level::Debug));
    EXPECT_NO_THROW(logger.setLevel(Logger::Level::Error));
}

TEST_F(LoggerTest, LoggingMethodsDoNotThrow) {
    Logger logger("test_logger");
    
    EXPECT_NO_THROW(logger.trace("Trace message"));
    EXPECT_NO_THROW(logger.debug("Debug message"));
    EXPECT_NO_THROW(logger.info("Info message"));
    EXPECT_NO_THROW(logger.warning("Warning message"));
    EXPECT_NO_THROW(logger.error("Error message"));
    EXPECT_NO_THROW(logger.critical("Critical message"));
}

TEST_F(LoggerTest, EmptyMessagesHandled) {
    Logger logger("test_logger");
    
    EXPECT_NO_THROW(logger.info(""));
    EXPECT_NO_THROW(logger.error(""));
    EXPECT_NO_THROW(logger.debug(""));
}

TEST_F(LoggerTest, LongMessagesHandled) {
    Logger logger("test_logger");
    std::string longMessage(5000, 'X');
    
    EXPECT_NO_THROW(logger.info(longMessage));
}

TEST_F(LoggerTest, SpecialCharactersHandled) {
    Logger logger("test_logger");
    
    EXPECT_NO_THROW(logger.info("Special chars: !@#$%^&*()"));
    EXPECT_NO_THROW(logger.info("Unicode: äöüß"));
    EXPECT_NO_THROW(logger.info("Control chars: \n\t\r"));
}

// Parameterized test for all log levels
TEST_P(LoggerLevelTest, AllLevelsWork) {
    Logger::Level level = GetParam();
    EXPECT_NO_THROW(logger_.setLevel(level));
    
    // Test that all logging methods work with any level set
    EXPECT_NO_THROW(logger_.trace("Test"));
    EXPECT_NO_THROW(logger_.debug("Test"));
    EXPECT_NO_THROW(logger_.info("Test"));
    EXPECT_NO_THROW(logger_.warning("Test"));
    EXPECT_NO_THROW(logger_.error("Test"));
    EXPECT_NO_THROW(logger_.critical("Test"));
}

INSTANTIATE_TEST_SUITE_P(
    LoggerLevels,
    LoggerLevelTest,
    ::testing::Values(
        Logger::Level::Trace,
        Logger::Level::Debug,
        Logger::Level::Info,
        Logger::Level::Warning,
        Logger::Level::Error,
        Logger::Level::Critical
    )
);

// Global logger tests
class GlobalLoggerTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Each test starts fresh
    }
    
    void TearDown() override {
        // Cleanup would go here if needed
    }
};

TEST_F(GlobalLoggerTest, InitializationWorks) {
    EXPECT_NO_THROW(initializeGlobalLogger("global_test"));
    EXPECT_NO_THROW(getGlobalLogger());
}

TEST_F(GlobalLoggerTest, GlobalLoggerFunctionality) {
    initializeGlobalLogger("global_func_test", Logger::Level::Debug);
    auto& logger = getGlobalLogger();
    
    EXPECT_NO_THROW(logger.info("Global logger test"));
    EXPECT_NO_THROW(logger.setLevel(Logger::Level::Warning));
    EXPECT_NO_THROW(logger.error("Error message"));
}

// Death tests for error conditions
TEST(LoggerDeathTest, UninitializedGlobalLoggerThrows) {
    // Note: This test assumes the global logger is not initialized
    // In a real test suite, you might want to reset the global state
    EXPECT_THROW(getGlobalLogger(), std::runtime_error);
}

// Performance benchmark test
TEST_F(LoggerTest, PerformanceBenchmark) {
    Logger logger("perf_test");
    logger.setLevel(Logger::Level::Info);
    
    auto start = std::chrono::high_resolution_clock::now();
    
    const int iterations = 10000;
    for (int i = 0; i < iterations; ++i) {
        logger.info("Benchmark message {}", i);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    // This is more of a benchmark than a test - it should complete in reasonable time
    EXPECT_LT(duration.count(), 5000); // Should complete in less than 5 seconds
}

// Test fixture for resource management
class LoggerResourceTest : public ::testing::Test {
  protected:
    void SetUp() override {
        logger_ = std::make_unique<Logger>("resource_test");
    }
    
    void TearDown() override {
        logger_.reset();
    }
    
    std::unique_ptr<Logger> logger_;
};

TEST_F(LoggerResourceTest, ResourceManagement) {
    ASSERT_NE(logger_, nullptr);
    EXPECT_NO_THROW(logger_->info("Resource test"));
    
    // Test that the logger can be reset without issues
    EXPECT_NO_THROW(logger_.reset());
    EXPECT_EQ(logger_, nullptr);
}