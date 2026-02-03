#include "point_cloud_process.h"
#include "compute_distance.h"
#include "log.h"
#include "apd_control.h"
#include <string>
#include <algorithm>
#include <chrono>
#include <mutex>
#include "user_api.hpp"
#include <opencv2/opencv.hpp>
#include "util.h"
#include "task_reg.h"

#include <Eigen/Core>
#include <open3d/Open3D.h>

using namespace open3d;
using namespace std;

// Parse depth and intensity from cv::Mat to std::vectors
void ParseDepthAndIntensity(
    const cv::Mat& src,
    std::vector<uint16_t>& depth,
    std::vector<uint16_t>& intensity)
{
    if (src.empty() || src.type() != CV_32SC1) {
        Logger::instance().error("Input Mat is empty or not CV_32SC1 in ParseDepthAndIntensity");
        return;
    }

    const int rows = src.rows;
    const int cols = src.cols;
    const int total_pixels = rows * cols;
    
    depth.resize(total_pixels);
    intensity.resize(total_pixels);

    for (int i = 0; i < rows; ++i) {
        const int32_t* ptr = src.ptr<int32_t>(i);
        for (int j = 0; j < cols; ++j) {
            uint32_t val = static_cast<uint32_t>(ptr[j]);

            depth[i * cols + j] = static_cast<uint16_t>((val >> 16) & 0xFFFF);
            intensity[i * cols + j] = static_cast<uint16_t>(val & 0xFFFF);
        }
    }
}

// Calculate distance from ToF matrix
std::vector<float> TimeToDistance(
    const std::vector<uint16_t>& matrix,
    int rows, int cols,
    int timeDelay)
{
    if (matrix.empty() || rows <= 0 || cols <= 0) {
        Logger::instance().error("Empty input matrix in TimeToDistance");
        return {};
    }

    Logger::instance().debug(("Thread3 - " + std::to_string(timeDelay) + " ns delay").c_str());

    std::vector<float> result(rows * cols);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int idx = i * cols + j;
            // 50 ~ 7950 range check
            if (matrix[idx] < 50  || matrix[idx] > 7950) {
                result[idx] = 0.0f; // If time is INVALID, distance is 0
                continue;
            }

            float distance = (16000-2*matrix[idx]) * 0.15f + timeDelay * 0.15f;
            result[idx] = distance;
        }
    }

    return result;
}

// Distance matrix to Point Cloud
void DistanceToPointcloud(const std::vector<float> &distance_matrix, int rows, int cols, std::vector<Eigen::Vector3d> &points)
{
    Logger::instance().debug("Converting distance matrix to point cloud");

    if (distance_matrix.empty() || rows <= 0 || cols <= 0) {
        Logger::instance().error("Empty input matrix in DistanceToPointcloud");
        return;
    }

    points.reserve(rows * cols);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
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

// Combine conversion and denoising
void ProcessAndDenoisePointCloud(
    const std::vector<float>& distance_matrix,
    const std::vector<uint16_t>& intensity_matrix,
    int rows, int cols,
    double eps,
    int min_points,
    std::vector<float>& denoised_distance,
    std::vector<uint16_t>& denoised_intensity,
    long long &duration_ms)
{
    auto start = std::chrono::high_resolution_clock::now();
    Logger::instance().debug("Starting ProcessAndDenoisePointCloud");

    // Only use distance info for conversion
    std::vector<Eigen::Vector3d> points;
    DistanceToPointcloud(distance_matrix, rows, cols, points);

    // Create point cloud object
    auto cloud = std::make_shared<geometry::PointCloud>();
    cloud->points_ = points;

    Logger::instance().debug(("point cloud size: " + std::to_string(cloud->points_.size())).c_str());

    // Denoise
    auto denoised_cloud = DenoiseWithDBSCAN(cloud, eps, min_points);

    Logger::instance().debug(("Denoised point cloud size: " + std::to_string(denoised_cloud->points_.size())).c_str());

    // Ensure output vectors serve as matrix, initialized to 0
    denoised_distance.assign(rows * cols, 0.0f);
    denoised_intensity.assign(rows * cols, 0);

    // Map back to matrix
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

template<typename T>
void FillHolesDilate(const std::vector<T>& src, std::vector<T>& dst, int rows, int cols, int kernal_size)
{
    cv::Mat src_mat(rows, cols, cv::DataType<T>::type, const_cast<T*>(src.data()));
    cv::Mat dilated;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernal_size, kernal_size));
    
    cv::dilate(src_mat, dilated, kernel);

    // Ensure dst is sized correctly
    if (dst.size() != src.size()) {
       dst.resize(src.size());
    }

    const T* dilated_data = reinterpret_cast<const T*>(dilated.data);
    for (int i = 0; i < rows * cols; ++i) {
        if (src[i] == static_cast<T>(0)) {
            dst[i] = dilated_data[i];
        } else {
            dst[i] = src[i];
        }
    }
}

// Helper to convert float vector to uint16 vector
std::vector<uint16_t> Float2Uint16(const std::vector<float>& floatArray) {
    std::vector<uint16_t> result(floatArray.size());
    for (size_t i = 0; i < floatArray.size(); ++i) {
        result[i] = static_cast<uint16_t>(floatArray[i]);
    }
    return result;
}

// Point Cloud Process Main Function
void PointCloudProcess(
    int timedelay,
    const cv::Mat& pcieMat,
    std::vector<float>& denoised_distance,
    std::vector<uint16_t>& denoised_intensity)
{
    Logger::instance().debug("Starting point cloud processing from PCIE cv::Mat");

    if (pcieMat.empty() || pcieMat.type() != CV_32SC1) {
        Logger::instance().error("Invalid or empty input cv::Mat for PointCloudProcess");
        return ;
    }

    int rows = pcieMat.rows;
    int cols = pcieMat.cols;

    ImagingAlgorithmParams paramsSnapshot;
    {
        std::lock_guard<std::mutex> lock(g_imagingParamMutex);
        paramsSnapshot = g_imagingParams;
    }
    
    const double dbscan_eps = paramsSnapshot.dbscanEps;
    const int dbscan_min_points = paramsSnapshot.dbscanMinSamples;
    const int reconstructionStride = paramsSnapshot.reconstructionStride;
    const float minValidDistance = paramsSnapshot.reconstructionThreshold;
    const int completionKernelSize = paramsSnapshot.completionKernelSize;

    Logger::instance().debug(("PointCloudProcess - eps: " + std::to_string(dbscan_eps) +
                              ", minPts: " + std::to_string(dbscan_min_points) +
                              ", stride: " + std::to_string(reconstructionStride) +
                              ", minDistance: " + std::to_string(minValidDistance) +
                              ", kernel: " + std::to_string(completionKernelSize)).c_str());

    std::vector<uint16_t> time_matrix;
    std::vector<uint16_t> intensity_matrix;

    ParseDepthAndIntensity(pcieMat, time_matrix, intensity_matrix);

    std::vector<float> distanceMatrix = TimeToDistance(time_matrix, rows, cols, timedelay);

    long long duration_ms = 0;

    ProcessAndDenoisePointCloud(distanceMatrix, intensity_matrix, rows, cols,
                                dbscan_eps, dbscan_min_points,
                                denoised_distance, denoised_intensity, duration_ms);

    FillHolesDilate<float>(denoised_distance, denoised_distance, rows, cols, completionKernelSize);
    FillHolesDilate<uint16_t>(denoised_intensity, denoised_intensity, rows, cols, completionKernelSize);
        
    return;
}

