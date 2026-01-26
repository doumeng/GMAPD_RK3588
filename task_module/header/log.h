#pragma once

#ifndef DEFAULT_LOG_LEVEL
#define DEFAULT_LOG_LEVEL INFO
#endif

#include <fstream>
#include <mutex>
#include <memory>

enum LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

class Logger {
public:
    // 单例获取
    static Logger& instance();
    static void init(const std::string& filename);

    void setLogLevel(LogLevel level);
    void debug(const char* message);
    void info(const char* message);
    void warning(const char* message);
    void error(const char* message);
    void critical(const char* message);

    ~Logger(); 

private:
    Logger(); 
    Logger(const std::string& filename);
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void log(LogLevel level, const char* message);
    const char* levelToString(LogLevel level);

    LogLevel currentLevel = DEFAULT_LOG_LEVEL;
    std::ofstream log_file_;
    std::mutex mtx_;

    static std::unique_ptr<Logger> s_instance_;
};
