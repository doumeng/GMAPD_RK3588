#include "point_cloud_process.h"
#include "compute_distance.h"
#include "log.h"
#include <string>
#include <algorithm>
#include <chrono>
#include "user_api.hpp"
#include <opencv2/opencv.hpp>
#include "util.h"

#include <Eigen/Core>
#include <open3d/Open3D.h>

using namespace open3d;
using namespace std;

// 从cv::Mat解析深度和强度到二维数组
void ParseDepthAndIntensity(
    const cv::Mat& src,
    uint16_t* depth,
    uint16_t* intensity)
{
    if (src.empty() || src.type() != CV_32SC1) {
        Logger::instance().error("Input Mat is empty or not CV_32SC1 in ParseDepthAndIntensity");
        return;
    }

    if (!depth || !intensity) {
        Logger::instance().error("Output pointers are null in ParseDepthAndIntensity");
        throw std::invalid_argument("Output pointers cannot be null");
    }

    const int rows = src.rows;
    const int cols = src.cols;

    for (int i = 0; i < rows; ++i) {
        const int32_t* ptr = src.ptr<int32_t>(i);
        for (int j = 0; j < cols; ++j) {
            uint32_t val = static_cast<uint32_t>(ptr[j]);

            depth[i * cols + j] = static_cast<uint16_t>((val >> 16) & 0xFFFF);
            intensity[i * cols + j] = static_cast<uint16_t>(val & 0xFFFF);
        }
    }
}

// 根据飞行时间计算距离
float* TimeToDistance(
    uint16_t* matrix, 
    int rows, int cols,
    int timeDelay)
{
    if (!matrix || rows <= 0 || cols <= 0) {
        Logger::instance().error("Empty input matrix in TimeToDistance");
    }

    Logger::instance().debug(("Thread3 - " + std::to_string(timeDelay) + " ns delay").c_str());

    float* result = createFloatMatrix(rows, cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // 测试验证
            if (matrix[i * cols + j] < 50  || matrix[i * cols + j] > 7950) {
                result[i * cols + j] = 0.0f; // 如果时间为0，距离也为0
                continue;
            }

            float distance = (16000-2*matrix[i * cols + j]) * 0.15f + timeDelay * 0.15f;
            result[i * cols + j] = distance;
        }
    }

    return result;
}

// 距离矩阵转点云
void DistanceToPointcloud(float* distance_matrix, int rows, int cols, std::vector<Eigen::Vector3d> &points)
{
    Logger::instance().debug("Converting distance matrix to point cloud");

    if (!distance_matrix || rows <= 0 || cols <= 0) {
        Logger::instance().error("Empty input matrix in DistanceToPointcloud");
    }

    points.reserve(rows * cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            float distance = distance_matrix[i * cols + j];
            if (distance > 0) {
                points.emplace_back(Eigen::Vector3d(i, j, distance));
            }
        }
    }
}

// DBSCAN去噪
shared_ptr<geometry::PointCloud> DenoiseWithDBSCAN(
    const shared_ptr<geometry::PointCloud> &cloud,
    double eps, int min_points)
{
    auto labels = cloud->ClusterDBSCAN(eps, min_points, false);

    auto denoised_cloud = make_shared<geometry::PointCloud>();
    for (size_t i = 0; i < labels.size(); ++i)
    {
        if (labels[i] != -1)
        {
            denoised_cloud->points_.push_back(cloud->points_[i]);
        }
    }

    return denoised_cloud;
}

// 合并点云转换和降噪，返回降噪后的距离和强度矩阵
void ProcessAndDenoisePointCloud(
    float* distance_matrix,
    uint16_t* intensity_matrix,
    int rows, int cols,
    double eps,
    int min_points,
    float* denoised_distance,
    uint16_t* denoised_intensity,
    long long &duration_ms)
{
    auto start = std::chrono::high_resolution_clock::now();
    Logger::instance().debug("Starting ProcessAndDenoisePointCloud");

    // 仅使用距离信息转换点云
    std::vector<Eigen::Vector3d> points;
    DistanceToPointcloud(distance_matrix, rows, cols, points);

    // 创建点云对象
    auto cloud = std::make_shared<geometry::PointCloud>();
    cloud->points_ = points;

    Logger::instance().debug(("point cloud size: " + std::to_string(cloud->points_.size())).c_str());

    // 降噪处理
    auto denoised_cloud = DenoiseWithDBSCAN(cloud, eps, min_points);

    Logger::instance().debug(("Denoised point cloud size: " + std::to_string(denoised_cloud->points_.size())).c_str());

    // 将降噪后的点云转换回矩阵形式，并过滤原始强度矩阵
    for (size_t i = 0; i < denoised_cloud->points_.size(); ++i) {
        const auto& point = denoised_cloud->points_[i];
        int x = static_cast<int>(point[0]);
        int y = static_cast<int>(point[1]);
        
        if (x >= 0 && x < rows && y >= 0 && y < cols) {
            denoised_distance[x * cols + y] = static_cast<float>(point[2]);
            denoised_intensity[x * cols + y] = intensity_matrix[x * cols + y];
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    Logger::instance().debug(("Process And DenoisePointCloud completed in " + std::to_string(duration_ms) + " ms").c_str());
}

// 简单形态学膨胀填充深度图中的 0 值孔洞
template<typename T>
void FillHolesDilate(const T* src, T* dst, int rows, int cols, int kernal_size)
{
    cv::Mat src_mat(rows, cols, cv::DataType<T>::type, const_cast<T*>(src));
    cv::Mat dilated;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernal_size, kernal_size));
    
    cv::dilate(src_mat, dilated, kernel);

    const T* dilated_data = reinterpret_cast<const T*>(dilated.data);
    for (int i = 0; i < rows * cols; ++i) {
        if (src[i] == static_cast<T>(0)) {
            dst[i] = dilated_data[i];
        } else if (src != dst) {
            dst[i] = src[i];
        }
    }
}

// 点云处理主函数
void PointCloudProcess(
    int timedelay,
    const cv::Mat& pcieMat,
    float * denoised_distance,
    uint16_t * denoised_intensity)
{
    Logger::instance().debug("Starting point cloud processing from PCIE cv::Mat");
    if (!denoised_distance || !denoised_intensity)
    {
        Logger::instance().error("Output pointers are null");
        return;
    }

    if (pcieMat.empty() || pcieMat.type() != CV_32SC1) {
        Logger::instance().error("Invalid or empty input cv::Mat for PointCloudProcess");
        return ;
    }

    int rows = pcieMat.rows;
    int cols = pcieMat.cols;

    // 分配二维数组
    uint16_t* time_matrix = createUint16Matrix(rows, cols);
    uint16_t* intensity_matrix = createUint16Matrix(rows, cols);

    ParseDepthAndIntensity(pcieMat, time_matrix, intensity_matrix);

    float* distanceMatrix = TimeToDistance(time_matrix, rows, cols, timedelay);

    // 转台测试
    // FillBadRowsAndCols(distanceMatrix, rows, cols);
    // FillBadRowsAndCols(intensity_matrix, rows, cols);

    // FillSmallHoles(distanceMatrix, distanceMatrix, rows, cols, 64);
    // FillSmallHoles(intensity_matrix, intensity_matrix, rows, cols, 64);

    // memcpy(denoised_distance, distanceMatrix, rows * cols * sizeof(float));
    // memcpy(denoised_intensity, intensity_matrix, rows * cols * sizeof(uint16_t));

    // 正常降噪
    const double dbscan_eps = 3;
    const int dbscan_min_points = 10;
    long long duration_ms = 0;

    ProcessAndDenoisePointCloud(distanceMatrix, intensity_matrix, rows, cols, 
                                dbscan_eps, dbscan_min_points,
                                denoised_distance, denoised_intensity, duration_ms);

    fillHolesDilate<float>(denoised_distance, denoised_distance, rows, cols, 5);
    fillHolesDilate<uint16_t>(denoised_intensity, denoised_intensity, rows, cols, 5);
        
    delete[] time_matrix;
    delete[] intensity_matrix;
    delete[] distanceMatrix;

    return;
}

