/*
 * @Author: doumeng 1159898567@qq.com
 * @Date: 2026-02-02 09:07:25
 * @LastEditors: doumeng 1159898567@qq.com
 * @LastEditTime: 2026-02-02 09:40:16
 * @FilePath: /GMAPD_RK3588/task_module/header/point_cloud_process.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

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

// 从cv::Mat解析深度和强度到std::vector
void ParseDepthAndIntensity(
    const cv::Mat &src,
    std::vector<uint16_t> &depth,
    std::vector<uint16_t> &intensity);

// 根据飞行时间计算距离
std::vector<float> TimeToDistance(
    const std::vector<uint16_t> &matrix,
    int rows, int cols,
    int timeDelay);

void DistanceToPointcloud(
    const std::vector<float> &distance_matrix,
    int rows, int cols,
    int stride,
    float minDistance,
    std::vector<Eigen::Vector3d> &points);

// 合并点云转换和降噪，返回降噪后的距离和强度矩阵
void ProcessAndDenoisePointCloud(
    const std::vector<float> &distance_matrix,
    const std::vector<uint16_t> &intensity_matrix,
    int rows, int cols,
    int stride,
    float min_valid_distance,
    double eps,
    int min_points,
    std::vector<float> &denoised_distance,
    std::vector<uint16_t> &denoised_intensity,
    long long &duration_ms);

template <typename T>
void FillHolesDilate(
    const std::vector<T>& src, 
    std::vector<T>& dst, 
    int rows, 
    int cols, 
    int kernal_size);

// Generate a new vector with uint16 distance from float distance
std::vector<uint16_t> Float2Uint16(const std::vector<float>& floatArray);

// 点云处理主函数
void PointCloudProcess(
    int timedelay,
    const cv::Mat &pcieMat,
    std::vector<float> &denoised_distance,
    std::vector<uint16_t> &denoised_intensity);