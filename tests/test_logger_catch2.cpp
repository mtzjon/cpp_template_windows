#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_session.hpp>

#include "cpp_project_template/logger.hpp"

#include <memory>
#include <stdexcept>

using namespace cpp_project_template;

TEST_CASE("Logger Construction", "[logger]") {
    SECTION("Valid logger creation") {
        REQUIRE_NOTHROW(Logger("test_logger"));
    }
    
    SECTION("Logger is movable") {
        Logger logger1("test_logger");
        Logger logger2 = std::move(logger1);
        REQUIRE_NOTHROW(logger2.info("Test message"));
    }
}

TEST_CASE("Logger Levels", "[logger]") {
    Logger logger("test_logger");
    
    SECTION("Set different log levels") {
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Trace));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Debug));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Info));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Warning));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Error));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Critical));
    }
}

TEST_CASE("Logger Methods", "[logger]") {
    Logger logger("test_logger");
    
    SECTION("Basic logging methods don't throw") {
        REQUIRE_NOTHROW(logger.trace("Trace message"));
        REQUIRE_NOTHROW(logger.debug("Debug message"));
        REQUIRE_NOTHROW(logger.info("Info message"));
        REQUIRE_NOTHROW(logger.warning("Warning message"));
        REQUIRE_NOTHROW(logger.error("Error message"));
        REQUIRE_NOTHROW(logger.critical("Critical message"));
    }
    
    SECTION("Empty messages are handled") {
        REQUIRE_NOTHROW(logger.info(""));
        REQUIRE_NOTHROW(logger.error(""));
    }
}

TEST_CASE("Global Logger", "[logger][global]") {
    SECTION("Global logger requires initialization") {
        // Reset any existing global logger
        // Note: In a real scenario, you might want to properly reset the global state
        REQUIRE_THROWS_AS(getGlobalLogger(), std::runtime_error);
    }
    
    SECTION("Global logger initialization and usage") {
        REQUIRE_NOTHROW(initializeGlobalLogger("global_test"));
        REQUIRE_NOTHROW(getGlobalLogger());
        
        auto& logger = getGlobalLogger();
        REQUIRE_NOTHROW(logger.info("Global logger test"));
        REQUIRE_NOTHROW(logger.setLevel(Logger::Level::Debug));
    }
}

TEST_CASE("Logger Edge Cases", "[logger][edge]") {
    SECTION("Very long log messages") {
        Logger logger("test_logger");
        std::string longMessage(10000, 'A');
        REQUIRE_NOTHROW(logger.info(longMessage));
    }
    
    SECTION("Special characters in messages") {
        Logger logger("test_logger");
        REQUIRE_NOTHROW(logger.info("Message with special chars: !@#$%^&*()"));
        REQUIRE_NOTHROW(logger.info("Unicode: üöäß"));
        REQUIRE_NOTHROW(logger.info("Newlines\nand\ttabs"));
    }
}