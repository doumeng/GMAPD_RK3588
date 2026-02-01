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


// 直方图统计结果结构体
struct HistogramResult {
    int maxPixelValue;  // 出现频数最高的像素值
    int maxFrequency;   // 该像素值出现的次数
    float occupancyRatio; // 非零像元比例
};

HistogramResult ComputeHistogram(
    const cv::Mat& image, 
    int startValue, 
    int endValue);

int ComputeDelay(
    float TargetDistance, 
    int BinWidth,  
    int Gatecount);