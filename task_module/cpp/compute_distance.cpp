#include "compute_distance.h"
#include "log.h"
#include <vector>
#include <string>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstring>
#include <tuple>

#if 0

// 直方图计算
void ComputeHistogram(const uint16_t *data, size_t dataLength, 
                    uint32_t *histogram, int &photonCount) {
    if (!data || !histogram) {
        logger.error("Invalid input parameters in computeHistogram");
        throw std::invalid_argument("Null pointer argument");
    }
    memset(histogram, 0, MAX_BIN * sizeof(uint32_t));
    photonCount = 0;

    for (size_t i = 0; i < dataLength; ++i) {
        if (data[i] <= MAX_BIN && data[i] > 0) {
            histogram[data[i]]++;
            photonCount++;
        }
    }
}

// 信噪比计算
void ComputeSNRPPP(uint32_t *histogram, int &photonCount,
                 float &noisePerGate, float &PPP, float &SNR) {
    if (!histogram) {
        logger.error("Null histogram in computeSNRPPP");
        throw std::invalid_argument("Null histogram");
    }
    int noise_count = 0;

    for (int i = 0; i < 4000; ++i) {
        noise_count += histogram[i];
    }

    noisePerGate = noise_count / 4000.0f;
    PPP = photonCount - noisePerGate * MAX_BIN;
    SNR = PPP / (noisePerGate * MAX_BIN);
    PPP /= (NUM_UNITS * MAX_BIN);
}

// 寻找阈值区间
std::pair<int, int> findLongestAboveThreshold(uint32_t *histogram, 
                                           float threshold) {
    int first = -1, last = -1;
    for (size_t i = 0; i < MAX_BIN; ++i) {
        if (histogram[i] > threshold) {
            first = i;
            break;
        }
    }

    for (size_t i = MAX_BIN - 1; i >= 0; --i) {
        if (histogram[i] > threshold) {
            last = i;
            break;
        }
    }

    return {first, last};
}

// 门控计算
std::tuple<int, int, float> ComputeGate(uint32_t *histogram,
                                      float &noisePerGate, float &SNR) {
    int max_Photon = 0;
    for (size_t i = 0; i < MAX_BIN; ++i) {
        if (histogram[i] > max_Photon) {
            max_Photon = histogram[i];
        }
    }

    float theta = noisePerGate + (max_Photon - noisePerGate) * (EPSILON / MY_GAMMA) * exp(-(EPSILON / MY_GAMMA) * SNR);
    auto [first, last] = findLongestAboveThreshold(histogram, theta);

    return {first, last, theta};
}

#endif

// 计算直方图并返回统计结果 (支持14位图像)
HistogramResult ComputeHistogram(const cv::Mat& image, int startValue, int endValue) {
    // 检查是否为空图像
    if (image.empty()) {
        return {-1, -1};
    }

    // 设置直方图参数（只统计100-4000范围内的像素值，共3901个bin）
    const int histSize = endValue - startValue + 1; // 3901
    const float range[] = {startValue, endValue+1};   // 上限值不包含
    const float* histRange = {range};

    // 计算直方图
    cv::Mat hist;
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);

    // 使用内置函数找到最大值及其位置
    cv::Point maxLoc;
    double maxValue;
    cv::minMaxLoc(hist, nullptr, &maxValue, nullptr, &maxLoc);

    // 计算 SNR
    // 简单近似: 峰值 / 平均背景噪声
    float totalCount = cv::sum(hist)[0];
    float peakCount = static_cast<float>(maxValue);
    float noiseTotal = totalCount - peakCount;
    float snr = 0.0f;
    if ((histSize - 1) > 0) {
        float avgNoise = noiseTotal / (histSize - 1);
        if (avgNoise > 1e-6) {
            snr = peakCount / avgNoise;
        } else {
            snr = (peakCount > 0) ? 9999.0f : 0.0f; 
        }
    }

    // maxLoc.y 对应的是 bin 索引，实际像素值为 bin + startValue
    return {maxLoc.y + startValue, static_cast<int>(maxValue), snr};
}

// 根据距离，计算需要的延迟时间
int ComputeDelay(float TargetDistance, int BinWidth, int Gatecount) {
    int bincount = (TargetDistance * 2) / (0.3 * BinWidth); // 计算需要的bin数
    int timeDelay = bincount - (bincount % Gatecount); // 计算实际延迟bin数
    return timeDelay * 2;
}

bool UpdateSettingsFromDistance(float distance, SystemConfig &config, HistConfig &histConfig) {
    return ;
}
