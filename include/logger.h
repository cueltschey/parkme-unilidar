#ifndef LOGGER_H
#define LOGGER_H

#include <cstdio>
#include <cstdarg>
#include <mutex>
#include <chrono>
#include <ctime>

typedef enum parkme_log_level_e { DEBUG = 0, INFO, WARNING, ERROR } parkme_log_level;

extern parkme_log_level log_level;

namespace parkme_logger {

constexpr const char* RESET   = "\033[0m";
constexpr const char* RED     = "\033[31m";
constexpr const char* YELLOW  = "\033[33m";
constexpr const char* GREEN   = "\033[32m";
constexpr const char* CYAN    = "\033[36m";

inline std::mutex log_mutex;

inline void log(parkme_log_level level, const char* fmt, ...) {
    const char* level_str = "";
    const char* color = "";

    if (level < log_level) return;

    switch (level) {
        case DEBUG:   level_str = "DEBUG";   color = CYAN; break;
        case INFO:    level_str = "INFO";    color = GREEN; break;
        case WARNING: level_str = "WARNING"; color = YELLOW; break;
        case ERROR:   level_str = "ERROR";   color = RED; break;
    }

    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char time_buf[20];
    std::strftime(time_buf, sizeof(time_buf), "%F %T", std::localtime(&t));

    std::lock_guard<std::mutex> lock(log_mutex);

    // Print log header with color
    std::fprintf(stderr, "[%s%s%s] [%s]: ", color, level_str, RESET, time_buf);

    // Handle variable arguments
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);

    // Reset color and newline
    std::fprintf(stderr, "\n");
    std::fflush(stderr);
}

}

#define LOG_DEBUG(fmt, ...)   parkme_logger::log(DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)    parkme_logger::log(INFO, fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...) parkme_logger::log(WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)   parkme_logger::log(ERROR, fmt, ##__VA_ARGS__)

#endif // LOGGER_H
