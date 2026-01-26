#pragma once
#include <opencv2/opencv.hpp>
#include <cstdint>
#include "log.h"

uint16_t * createUint16Matrix(int rows, int cols);
float * createFloatMatrix(int rows, int cols);

uint32_t * interleaveArrays(const float* floatArray, const uint16_t* intArray, size_t size);

// 将CV_32SC1的cv::Mat数据拷贝到一段连续内存，返回首地址（需手动释放）
uint32_t * MatToContiguousArray(const cv::Mat& mat, int& rows, int& cols);
uint16_t * float2Uint16(const float *floatArray, size_t size);

