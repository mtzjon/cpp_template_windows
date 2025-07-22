#include "cpp_project_template/logger.hpp"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <fmt/format.h>

#include <stdexcept>

namespace cpp_project_template {

namespace {
// Convert our enum to spdlog level
spdlog::level::level_enum toSpdlogLevel(Logger::Level level) {
    switch (level) {
        case Logger::Level::Trace:
            return spdlog::level::trace;
        case Logger::Level::Debug:
            return spdlog::level::debug;
        case Logger::Level::Info:
            return spdlog::level::info;
        case Logger::Level::Warning:
            return spdlog::level::warn;
        case Logger::Level::Error:
            return spdlog::level::err;
        case Logger::Level::Critical:
            return spdlog::level::critical;
        default:
            return spdlog::level::info;
    }
}

// Global logger instance
std::unique_ptr<Logger> g_logger;
}  // namespace

Logger::Logger(std::string_view name) {
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

template <typename... Args>
void Logger::log(Level level, fmt::format_string<Args...> format, Args&&... args) {
    if (logger_) {
        const auto message = fmt::format(format, std::forward<Args>(args)...);
        logger_->log(toSpdlogLevel(level), message);
    }
}

// Explicit instantiations for common use cases
template void Logger::log<>(Level, fmt::format_string<>, ...);
template void Logger::log<int>(Level, fmt::format_string<int>, int&&);
template void Logger::log<std::string>(Level, fmt::format_string<std::string>, std::string&&);
template void Logger::log<const char*>(Level, fmt::format_string<const char*>, const char*&&);

Logger& getGlobalLogger() {
    if (!g_logger) {
        throw std::runtime_error("Global logger not initialized. Call initializeGlobalLogger() first.");
    }
    return *g_logger;
}

void initializeGlobalLogger(std::string_view name, Logger::Level level) {
    g_logger = std::make_unique<Logger>(name);
    g_logger->setLevel(level);
}

}  // namespace cpp_project_template