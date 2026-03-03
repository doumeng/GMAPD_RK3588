/*
 * @Author: doumeng 1159898567@qq.com
 * @Date: 2026-01-27 09:00:23
 * @LastEditors: doumeng 1159898567@qq.com
 * @LastEditTime: 2026-03-02 14:44:57
 * @FilePath: /GMAPD_RK3588/task_module/header/log.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#ifndef DEFAULT_LOG_LEVEL
#define DEFAULT_LOG_LEVEL DEBUG
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
