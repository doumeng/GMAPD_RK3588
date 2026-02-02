#pragma once

#include <array>
#include <string>
#include <cstddef>
#include "task_reg.h" 

constexpr size_t kDistanceBinCount = 7;
constexpr size_t kSparsityLevelCount = 5;

using ImagingParamLUT = std::array<std::array<ImagingAlgorithmParams, kSparsityLevelCount>, kDistanceBinCount>;

const ImagingParamLUT &ImagingParamLut();

ImagingAlgorithmParams MakeDefaultImagingParams();

size_t ResolveDistanceIndex(float distance);

size_t ResolveSparsityIndex(float occupancyRatio);
