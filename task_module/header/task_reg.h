#pragma once
#include <shared_mutex>
#include <mutex>

// 共享数据结构
struct SharedData
{
    bool dataUpdated = false;
    float distance = 0.0f;      // 弹目距离
    float occupancyRatio = 0.0f; // 非零像元比例
    int timedelay = 0;       // 延时
    float velocity = 0.0f;
};

struct SharedMat
{
    std::shared_mutex matMutex;
    uint16_t * sharedMat = new uint16_t[128 * 128];
    bool newDataAvailable = false;
};

struct ImagingAlgorithmParams
{
    int tofFrameCount = 64;
    int reconstructionStride = 1;
    float reconstructionThreshold = 120.0f;
    double dbscanEps = 3.0;
    int dbscanMinSamples = 10;
    int completionKernelSize = 5;
};

extern ImagingAlgorithmParams g_imagingParams;
extern std::mutex g_imagingParamMutex;

// 线程注册接口
void register_threads();