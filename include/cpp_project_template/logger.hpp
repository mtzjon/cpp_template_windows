#pragma once

#include <memory>
#include <string>
#include <string_view>

#include <fmt/format.h>

namespace spdlog {
class logger;
namespace level {
enum level_enum : int;
} // namespace level
} // namespace spdlog

namespace cpp_project_template {

/**
 * @brief A modern C++ logging utility wrapper around spdlog
 *
 * This class provides a clean interface for logging functionality
 * with various log levels and formatting support.
 */
class Logger {
  public:
    /**
     * @brief Log levels supported by the logger
     */
    enum class Level { TRACE, DEBUG, INFO, WARN, ERR, CRITICAL };

    /**
     * @brief Constructor
     * @param name The name of the logger
     */
    explicit Logger(std::string_view name);

    /**
     * @brief Destructor
     */
    ~Logger();

    // Make Logger non-copyable but movable
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) noexcept;
    Logger& operator=(Logger&&) noexcept;

    /**
     * @brief Set the logging level
     * @param level The minimum level to log
     */
    void setLevel(Level level);

    /**
     * @brief Log a message at trace level
     * @param message The message to log
     */
    void trace(std::string_view message);

    /**
     * @brief Log a formatted message at trace level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void trace(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Log a message at debug level
     * @param message The message to log
     */
    void debug(std::string_view message);

    /**
     * @brief Log a formatted message at debug level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void debug(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Log a message at info level
     * @param message The message to log
     */
    void info(std::string_view message);

    /**
     * @brief Log a formatted message at info level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void info(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Log a message at warning level
     * @param message The message to log
     */
    void warning(std::string_view message);

    /**
     * @brief Log a formatted message at warning level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void warning(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Log a message at error level
     * @param message The message to log
     */
    void error(std::string_view message);

    /**
     * @brief Log a formatted message at error level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void error(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Log a message at critical level
     * @param message The message to log
     */
    void critical(std::string_view message);

    /**
     * @brief Log a formatted message at critical level
     * @tparam Args Variadic template arguments
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void critical(fmt::format_string<Args...> format, Args&&... args);

    /**
     * @brief Format and log a message
     * @tparam Args Variadic template arguments
     * @param level The log level
     * @param format The format string
     * @param args The arguments to format
     */
    template <typename... Args>
    void log(Level level, fmt::format_string<Args...> format, Args&&... args);

  private:
    std::shared_ptr<spdlog::logger> logger_;

    // Helper function to convert our enum to spdlog level
    static spdlog::level::level_enum toSpdlogLevel(Level level);
};

/**
 * @brief Get the global logger instance
 * @return Reference to the global logger
 */
Logger& getGlobalLogger();

/**
 * @brief Initialize the global logger
 * @param name The name for the global logger
 * @param level The initial log level
 */
void initializeGlobalLogger(std::string_view name, Logger::Level level = Logger::Level::INFO);

}  // namespace cpp_project_template
