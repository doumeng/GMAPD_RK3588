/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-01-27 09:00:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-27 15:21:49
 * @FilePath: /GMAPD_RK3588/task_module/cpp/compute_distance.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "compute_distance.h"
#include "log.h"
#include <vector>
#include <string>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstring>
#include <tuple>

// 计算直方图并返回统计结果 (支持14位图像)
HistogramResult ComputeHistogram(const cv::Mat& image, int startValue, int endValue) {
    // 检查是否为空图像
    if (image.empty()) {
        return {-1, -1};
    }

    // 设置直方图参数（只统计1000-50000范围内的像素值）
    const int histSize = static_cast<int>((endValue - startValue + 1) / 1000);
    const float range[] = {startValue, endValue+1};   // 上限值不包含
    const float* histRange = {range};

    // 计算直方图
    cv::Mat hist;
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true);

    // 使用内置函数找到最大值及其位置
    cv::Point maxLoc;
    double maxValue;
    cv::minMaxLoc(hist, nullptr, &maxValue, nullptr, &maxLoc);

    // 计算非零像素占比，来近似得到信噪比
    double totalPixels = cv::countNonZero(image);
    double signalPixels = maxValue;
    float snr = (totalPixels > 0) ? static_cast<float>(signalPixels / totalPixels) : 0.0f;

    // maxLoc.y 对应的是 bin 索引，实际像素值为 bin + startValue
    return {maxLoc.y + startValue, static_cast<int>(maxValue), snr};
}

// 根据距离，计算需要的延迟时间
int ComputeDelay(float TargetDistance, int BinWidth, int Gatecount) {
    int bincount = (TargetDistance * 2) / (0.3 * BinWidth); // 计算需要的bin数
    int timeDelay = bincount - (bincount % Gatecount); // 计算实际延迟bin数
    return timeDelay * 2;
}
