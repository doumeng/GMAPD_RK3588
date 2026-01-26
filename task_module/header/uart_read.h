/*
 * @Author: doumeng
 * @Date: 2025-05-30 19:52:42
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-26 16:00:08
 * @FilePath: uart_read.h
 * @Description: 读取串口信息并解析至对应的结构体
*/

#pragma once
#include <mutex>
#include <cstdint>
#include <cstddef>
#include "log.h"

// 工作模式枚举
enum class WorkMode
{
    STANDARD,                                       // 正常工作模式 0
    TEST                                            // 测试模式 1
};

// 探测器触发模式枚举
enum class TriggerMode
{
    INTERNAL,                                       // 内触发 0
    EXTERNAL                                        // 外触发 1
};

// 指令类型
enum class UartCmdType
{
    NONE,
    PARAM_CONFIG,                                   // 0xC4 参数配置
    MOTION_INFO,                                    // 0xC6 弹体信息
    APD_Timing_START,                               // 0xC7 APD时序启动指令
    STANDBY,                                        // 0xCB 待机指令
    APD_STOP                                        // 0xCC 下电指令
};

// 系统配置参数
struct SystemConfig
{
    WorkMode workMode = WorkMode::STANDARD;          // 工作模式
    TriggerMode triggerMode = TriggerMode::EXTERNAL; // 探测器触发模式
    float timeResolution = 2.0f;                     // 时间分辨率(ns)
    float biasVoltage = 0.0f;                        // 电压(V)
    int16_t enDelay = 0;                             // 开门延迟（ns）
};

struct HistConfig
{
    int16_t stride = 8;                               // 系统配置
    int16_t threshold = 2;                            // 阈值
};

struct PostprocessConfig
{
    double eps = 5.0f;                                 // 邻域半径
    int minSamples = 5;                                // 最小样本数
    int kernalSize = 5;                                 // 高斯核大小
};

// 弹体数据
struct MotionData
{
    float velocity = 200.0f;                           // 速度(m/s)
    float distance = 1500.0f;                          // 距离(m)
    float roll = 0.0f;                                 // 横滚角(度)
    float pitch = 0.0f;                                // 俯仰角(度)
    float yaw = 0.0f;                                  // 偏航角(度)
};

struct EnvConfig{
    float snr = 10.0f;                                   // 信噪比
};

UartCmdType parseUartCommand(
    const char *buf, 
    size_t len,
    SystemConfig &sysCfg,
    HistConfig &histCfg,
    MotionData &motionData);
