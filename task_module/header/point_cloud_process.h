
#pragma once

#include <vector>
#include <string>
#include <memory>
#include <tuple>
#include <opencv2/opencv.hpp>
#include "util.h" // 包含二维数组分配和释放的函数声明
#include "log.h"  // 日志记录功能

#include <Eigen/Core>
#include <open3d/Open3D.h>

// 二维数组内存分配和释放由 util.h 提供

// 从cv::Mat解析深度和强度到二维数组
void ParseDepthAndIntensity(
    const cv::Mat &src,
    uint16_t *depth,
    uint16_t *intensity);

// 根据飞行时间计算距离
float *TimeToDistance(
    float *matrix,
    int rows, int cols,
    int timeDelay);

// 计算中央区域的最大、最小、平均距离
// std::tuple<float, float, float> ComputeDisAndDoF(
//     float* depth_map,
//     int rows, int cols);
// // 距离矩阵转点云
void DistanceToPointcloud(
    float *distance_matrix,
    int rows, int cols,
    int stride,
    float minDistance,
    std::vector<Eigen::Vector3d> &points);

// 合并点云转换和降噪，返回降噪后的距离和强度矩阵
void ProcessAndDenoisePointCloud(
    float *distance_matrix,
    uint16_t *intensity_matrix,
    int rows, int cols,
    int stride,
    float min_valid_distance,
    double eps,
    int min_points,
    float *denoised_distance,
    uint16_t *denoised_intensity,
    long long &duration_ms);

template <typename T>
void FillHolesDilate(
    const T* src, 
    T* dst, 
    int rows, 
    int cols, 
    int kernal_size);

// 点云处理主函数
void PointCloudProcess(
    int timedelay,
    const cv::Mat &pcieMat,
    float *denoised_distance,
    uint16_t *denoised_intensity);