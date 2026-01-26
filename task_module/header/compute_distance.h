#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <utility>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "log.h"

#define MY_GAMMA 1
#define EPSILON 0.5
#define Bin_Width 4 //ns

#define MAX_BIN 4096
#define NUM_UNITS 128 * 128

#if 0

void ComputeHistogram(const uint16_t *data, size_t dataLength, 
                     uint32_t *histogram, int &photonCount);

void ComputeSNRPPP(uint32_t *histogram, int &photonCount,
                 float &noisePerGate, float &PPP, float &SNR);

std::pair<int, int> findLongestAboveThreshold(uint32_t *histogram, 
                                            float threshold);

std::tuple<int, int, float> ComputeGate(uint32_t *histogram,
                                      float &noisePerGate, float &SNR);
#endif

// 直方图统计结果结构体
struct HistogramResult {
    int maxPixelValue;  // 出现频数最高的像素值
    int maxFrequency;   // 该像素值出现的次数
    float snr;          // 信噪比
};

HistogramResult ComputeHistogram(const cv::Mat& image, int startValue, int endValue);
int ComputeDelay(float TargetDistance, int BinWidth,  int Gatecount);