#include "cpp_project_template/logger.hpp"

#include <fmt/format.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <stdexcept>

namespace cpp_project_template {

namespace {
// Global logger instance
std::unique_ptr<Logger> g_logger;
}  // namespace

// Convert our enum to spdlog level
spdlog::level::level_enum Logger::toSpdlogLevel(Level level) {
    switch (level) {
        case Level::TRACE:
            return spdlog::level::trace;
        case Level::DEBUG:
            return spdlog::level::debug;
        case Level::INFO:
            return spdlog::level::info;
        case Level::WARN:
            return spdlog::level::warn;
        case Level::ERR:
            return spdlog::level::err;
        case Level::CRITICAL:
            return spdlog::level::critical;
        default:
            return spdlog::level::info;
    }
}

Logger::Logger(std::string_view name) {
    if (logger_ && name == logger_->name()) {
        return;
    }
    logger_ = spdlog::stdout_color_mt(std::string{name});
    if (!logger_) {
        throw std::runtime_error("Failed to create logger");
    }
    logger_->set_level(spdlog::level::info);
    logger_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
}

Logger::~Logger() = default;

Logger::Logger(Logger&&) noexcept = default;
Logger& Logger::operator=(Logger&&) noexcept = default;

void Logger::setLevel(Level level) {
    if (logger_) {
        logger_->set_level(toSpdlogLevel(level));
    }
}

void Logger::trace(std::string_view message) {
    if (logger_) {
        logger_->trace(message);
    }
}

void Logger::debug(std::string_view message) {
    if (logger_) {
        logger_->debug(message);
    }
}

void Logger::info(std::string_view message) {
    if (logger_) {
        logger_->info(message);
    }
}

void Logger::warning(std::string_view message) {
    if (logger_) {
        logger_->warn(message);
    }
}

void Logger::error(std::string_view message) {
    if (logger_) {
        logger_->error(message);
    }
}

void Logger::critical(std::string_view message) {
    if (logger_) {
        logger_->critical(message);
    }
}

// Template method implementations
template <typename... Args>
void Logger::trace(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::TRACE, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::debug(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::DEBUG, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::info(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::INFO, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::warning(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::WARN, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::error(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::ERR, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::critical(fmt::format_string<Args...> format, Args&&... args) {
    log(Level::CRITICAL, format, std::forward<Args>(args)...);
}

template <typename... Args>
void Logger::log(Level level, fmt::format_string<Args...> format, Args&&... args) {
    if (logger_) {
        const auto message = fmt::format(format, std::forward<Args>(args)...);
        logger_->log(toSpdlogLevel(level), message);
    }
}

// Explicit instantiations for common use cases
template void Logger::trace<int>(fmt::format_string<int>, int&&);
template void Logger::trace<double>(fmt::format_string<double>, double&&);
template void Logger::trace<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::trace<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::trace<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::trace<bool>(fmt::format_string<bool>, bool&&);

template void Logger::debug<int>(fmt::format_string<int>, int&&);
template void Logger::debug<double>(fmt::format_string<double>, double&&);
template void Logger::debug<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::debug<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::debug<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::debug<bool>(fmt::format_string<bool>, bool&&);

template void Logger::info<int>(fmt::format_string<int>, int&&);
template void Logger::info<double>(fmt::format_string<double>, double&&);
template void Logger::info<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::info<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::info<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::info<bool>(fmt::format_string<bool>, bool&&);
template void Logger::info<size_t>(fmt::format_string<size_t>, size_t&&);

template void Logger::warning<int>(fmt::format_string<int>, int&&);
template void Logger::warning<double>(fmt::format_string<double>, double&&);
template void Logger::warning<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::warning<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::warning<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::warning<bool>(fmt::format_string<bool>, bool&&);

template void Logger::error<int>(fmt::format_string<int>, int&&);
template void Logger::error<double>(fmt::format_string<double>, double&&);
template void Logger::error<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::error<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::error<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::error<bool>(fmt::format_string<bool>, bool&&);

template void Logger::critical<int>(fmt::format_string<int>, int&&);
template void Logger::critical<double>(fmt::format_string<double>, double&&);
template void Logger::critical<std::string>(fmt::format_string<std::string>, std::string&&);
template void Logger::critical<const char*>(fmt::format_string<const char*>, const char*&&);
template void Logger::critical<std::string_view>(fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::critical<bool>(fmt::format_string<bool>, bool&&);

template void Logger::log<int>(Level, fmt::format_string<int>, int&&);
template void Logger::log<double>(Level, fmt::format_string<double>, double&&);
template void Logger::log<std::string>(Level, fmt::format_string<std::string>, std::string&&);
template void Logger::log<const char*>(Level, fmt::format_string<const char*>, const char*&&);
template void Logger::log<std::string_view>(Level, fmt::format_string<std::string_view>, std::string_view&&);
template void Logger::log<bool>(Level, fmt::format_string<bool>, bool&&);
template void Logger::log<size_t>(Level, fmt::format_string<size_t>, size_t&&);

// Multi-argument templates for the cases in main.cpp
template void Logger::debug<double, size_t>(fmt::format_string<double, size_t>, double&&, size_t&&);
template void Logger::log<double, size_t>(Level, fmt::format_string<double, size_t>, double&&, size_t&&);

// Additional instantiations for const references and lvalue references
template void Logger::debug<const std::string&>(fmt::format_string<const std::string&>, const std::string&);
template void Logger::debug<std::string&>(fmt::format_string<std::string&>, std::string&);
template void Logger::info<const std::string&>(fmt::format_string<const std::string&>, const std::string&);
template void Logger::info<std::string&>(fmt::format_string<std::string&>, std::string&);
template void Logger::debug<const double&, size_t>(fmt::format_string<const double&, size_t>, const double&, size_t&&);
template void Logger::info<const double&>(fmt::format_string<const double&>, const double&);
template void Logger::info<int&>(fmt::format_string<int&>, int&);

Logger& getGlobalLogger() {
    if (!g_logger) {
        throw std::runtime_error(
            "Global logger not initialized. Call initializeGlobalLogger() first.");
    }
    return *g_logger;
}

void initializeGlobalLogger(std::string_view name, Logger::Level level) {
    g_logger = std::make_unique<Logger>(name);
    g_logger->setLevel(level);
}

}  // namespace cpp_project_template
