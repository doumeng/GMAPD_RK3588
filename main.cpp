#include <iostream>
#include <chrono>
#include <thread>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include "log.h"
#include "user_api.hpp"
#include <fstream>
#include <time.h>
#include "task_reg.h"

using namespace std;

int main()
{   
    std::string g_outputDir = "./log/"; // 输出目录

    // 获取当前时间，生成唯一日志文件名
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y%m%d_%H%M%S", t);
    std::string logFilePath = g_outputDir + "log_" + timebuf + ".txt";

    // 先创建目录
    if (mkdir(g_outputDir.c_str(), 0777) == -1 && errno != EEXIST) {
        std::cerr << "Failed to create output directory" << std::endl;
        return 1;
    }

    // 初始化 Logger
    Logger::init(logFilePath);
    Logger::instance().info("Program started");

    Logger::instance().info(("Output directory created: " + logFilePath).c_str());

    SysInit();

    Logger::instance().info("System initialized");

    try {
        // 注册并启动线程
        register_threads();
        Logger::instance().info("Program completed successfully");

    } catch (const std::exception& e) {
        Logger::instance().error("Program failed with exception");
        Logger::instance().error(e.what());
        return 1;
    }

    return 0;
}
