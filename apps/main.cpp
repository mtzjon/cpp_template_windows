#include "cpp_project_template/logger.hpp"

#include <CLI/CLI.hpp>
#include <fmt/format.h>

#include <iostream>
#include <string>
#include <vector>

using namespace cpp_project_template;

/**
 * @brief Main application entry point
 *
 * Demonstrates modern C++ features and the project's logging capabilities.
 */
int main(int argc, char* argv[]) {
    CLI::App app{"C++ Project Template - A modern C++ application example",
                 "cpp-project-template"};
    app.set_version_flag("--version", "1.0.0");

    // Command line options
    std::string logLevel = "info";
    std::string message = "Hello, Modern C++!";
    bool verbose = false;
    std::vector<std::string> inputs;

    app.add_option("-l,--log-level", logLevel,
                   "Set log level (trace,debug,info,warning,error,critical)")
        ->check(CLI::IsMember({"trace", "debug", "info", "warning", "error", "critical"}));

    app.add_option("-m,--message", message, "Custom message to display");
    app.add_flag("-v,--verbose", verbose, "Enable verbose output");
    app.add_option("inputs", inputs, "Input files to process");

    CLI11_PARSE(app, argc, argv);

    try {
        // Initialize the global logger
        initializeGlobalLogger("CppProjectTemplate");

        // Set log level based on command line argument
        Logger::Level level = Logger::Level::INFO;
        if (logLevel == "trace")
            level = Logger::Level::TRACE;
        else if (logLevel == "debug")
            level = Logger::Level::DEBUG;
        else if (logLevel == "info")
            level = Logger::Level::INFO;
        else if (logLevel == "warning")
            level = Logger::Level::WARN;
        else if (logLevel == "error")
            level = Logger::Level::ERR;
        else if (logLevel == "critical")
            level = Logger::Level::CRITICAL;

        auto& logger = getGlobalLogger();
        logger.setLevel(level);

        // Demonstrate logging at different levels
        logger.info("Application started successfully");
        logger.debug("Debug mode: {}", verbose ? "enabled" : "disabled");

        if (verbose) {
            logger.trace("Verbose mode enabled - showing detailed information");
            logger.debug("Command line arguments processed");
            logger.debug("Log level set to: {}", logLevel);
        }

        // Display the main message
        logger.info("Message: {}", message);

        // Process input files if provided
        if (!inputs.empty()) {
            logger.info("Processing {} input file(s):", inputs.size());
            for (const auto& input : inputs) {
                logger.info("  - {}", input);
                // Here you would typically process the file
                // This is just a demonstration
            }
        } else {
            logger.debug("No input files provided");
        }

        // Demonstrate modern C++ features
        auto processData = [&logger](const std::vector<int>& data) -> double {
            if (data.empty()) {
                logger.warning("Empty data set provided");
                return 0.0;
            }

            // Use C++20 ranges (simplified example)
            double sum = 0.0;
            for (const auto& value : data) {
                sum += value;
            }

            const double average = sum / static_cast<double>(data.size());
            logger.debug("Calculated average: {:.2f} from {} values", average, data.size());
            return average;
        };

        // Sample data processing
        const std::vector<int> sampleData{1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
        const double result = processData(sampleData);

        logger.info("Sample calculation result: {:.2f}", result);

        // Demonstrate error handling
        try {
            if (message.empty()) {
                throw std::invalid_argument("Message cannot be empty");
            }
        } catch (const std::exception& e) {
            logger.error("Error: {}", e.what());
            return 1;
        }

        logger.info("Application completed successfully");
        return 0;

    } catch (const std::exception& e) {
        std::cerr << fmt::format("Fatal error: {}\n", e.what());
        return 1;
    } catch (...) {
        std::cerr << "Unknown fatal error occurred\n";
        return 1;
    }
}
