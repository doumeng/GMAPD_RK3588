#pragma once
#include <shared_mutex>

// 共享数据结构
struct SharedData
{
    bool dataUpdated = false;
    float distance = 0.0f;      // 弹目距离
    float snr = 0.0f;           // 信噪比
    int timedelay = 0;       // 延时
    float velocity = 0.0f;
};

struct SharedMat
{
    std::shared_mutex matMutex;
    uint16_t * sharedMat = new uint16_t[128 * 128];
    bool newDataAvailable = false;
};

// 线程注册接口
void register_threads();