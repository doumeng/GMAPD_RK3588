#include "log.h"
#include <iostream>
#include <ctime>

std::unique_ptr<Logger> Logger::s_instance_ = nullptr;

Logger::Logger() {}

Logger::Logger(const std::string& filename) {
    log_file_.open(filename, std::ios::app);
    if (!log_file_.is_open()) {
        std::cerr << "Logger: Failed to open log file: " << filename << std::endl;
    }
}

Logger::~Logger() {
    if (log_file_.is_open()) {
        log_file_.close();
    }
}

void Logger::init(const std::string& filename) {
    if (s_instance_) {
        // 已经初始化过，忽略
        return;
    }
    s_instance_ = std::unique_ptr<Logger>(new Logger(filename));
}

Logger& Logger::instance() {
    if (!s_instance_) {
        // 如果还没初始化，给出警告并用默认文件名
        std::cerr << "Logger::instance() called before Logger::init(), using default log.txt" << std::endl;
        s_instance_ = std::unique_ptr<Logger>(new Logger("log.txt"));
    }
    return *s_instance_;
}

void Logger::setLogLevel(LogLevel level) {
    currentLevel = level;
}

const char* Logger::levelToString(LogLevel level) {
    switch (level) {
        case DEBUG: return "DEBUG";
        case INFO: return "INFO";
        case WARNING: return "WARNING";
        case ERROR: return "ERROR";
        case CRITICAL: return "CRITICAL";
        default: return "UNKNOWN";
    }
}

void Logger::log(LogLevel level, const char* message) {
    if (level < currentLevel) return;

    std::lock_guard<std::mutex> lock(mtx_);
    std::time_t t = std::time(nullptr);
    char timebuf[32];
    std::strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    std::string log_line = std::string("[") + timebuf + "][" + levelToString(level) + "] " + message;

    std::cout << log_line << std::endl;
    if (log_file_.is_open()) {
        log_file_ << log_line << std::endl;
    }
}

void Logger::debug(const char* message)    { log(DEBUG, message); }
void Logger::info(const char* message)     { log(INFO, message); }
void Logger::warning(const char* message)  { log(WARNING, message); }
void Logger::error(const char* message)    { log(ERROR, message); }
void Logger::critical(const char* message) { log(CRITICAL, message); }
