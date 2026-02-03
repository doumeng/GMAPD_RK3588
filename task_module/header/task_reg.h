/*
 * @Author: doumeng 1159898567@qq.com
 * @Date: 2026-02-02 09:07:25
 * @LastEditors: doumeng 1159898567@qq.com
 * @LastEditTime: 2026-02-02 15:39:14
 * @FilePath: /GMAPD_RK3588/task_module/header/task_reg.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <shared_mutex>
#include <mutex>
#include <condition_variable>

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
    int tofFrameCount = 4;
    int reconstructionStride = 3;
    int reconstructionThreshold = 2;
    double dbscanEps = 3.0;
    int dbscanMinSamples = 10;
    int completionKernelSize = 3;
};

extern ImagingAlgorithmParams g_imagingParams;
extern std::mutex g_imagingParamMutex;

// 线程注册接口
void register_threads();