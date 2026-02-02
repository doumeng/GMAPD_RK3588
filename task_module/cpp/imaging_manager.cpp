#include "imaging_manager.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>

namespace
{
    constexpr float kDistanceBinSizeMeters = 500.0f;
    constexpr float kMaxDistanceMeters = 3000.0f;
    constexpr float kSparsityStep = 0.2f;
    constexpr const char *kImagingParamCsvPath = "/userdata/dm/GMAPD_RK3588/imaging_params.csv";

    struct DistanceProfile
    {
        int tofFrameCount;
        int reconstructionStride;
        float reconstructionThreshold;
        double dbscanEps;
        int dbscanMinSamples;
        int kernelSize;
    };

    struct SparsityProfile
    {
        int tofFrameDelta;
        int strideDelta;
        float thresholdDelta;
        double epsDelta;
        int minSampleDelta;
        int kernelDelta;
    };

    std::string Trim(const std::string &input)
    {
        const auto start = input.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
        {
            return "";
        }
        const auto end = input.find_last_not_of(" \t\r\n");
        return input.substr(start, end - start + 1);
    }

    bool ParseCsvRow(const std::string &line,
                     size_t &distanceIdx,
                     size_t &sparsityIdx,
                     ImagingAlgorithmParams &params)
    {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;
        while (std::getline(ss, token, ','))
        {
            tokens.push_back(Trim(token));
        }

        if (tokens.size() < 7)
        {
            return false;
        }

        try
        {
            distanceIdx = static_cast<size_t>(std::stoi(tokens[0]));
            sparsityIdx = static_cast<size_t>(std::stoi(tokens[1]));
            params.tofFrameCount = std::stoi(tokens[2]);
            params.reconstructionStride = std::stoi(tokens[3]);
            params.reconstructionThreshold = std::stof(tokens[4]);
            params.dbscanEps = std::stod(tokens[5]);
            params.dbscanMinSamples = std::stoi(tokens[6]);
            if (tokens.size() > 7)
            {
                params.completionKernelSize = std::stoi(tokens[7]);
            }
        }
        catch (const std::exception &ex)
        {
            std::cerr << "ParseCsvRow - invalid value, line: " << line << ", reason: " << ex.what() << std::endl;
            return false;
        }

        params.tofFrameCount = std::clamp(params.tofFrameCount, 16, 256);
        params.reconstructionStride = std::clamp(params.reconstructionStride, 1, 4);
        params.reconstructionThreshold = std::clamp(params.reconstructionThreshold, 0.0f, 1000.0f);
        params.dbscanEps = std::max(0.1, params.dbscanEps);
        params.dbscanMinSamples = std::clamp(params.dbscanMinSamples, 1, 128);
        if (params.completionKernelSize <= 0)
        {
            params.completionKernelSize = 3;
        }
        if ((params.completionKernelSize % 2) == 0)
        {
            ++params.completionKernelSize;
        }
        params.completionKernelSize = std::clamp(params.completionKernelSize, 3, 15);

        return true;
    }

    bool LoadImagingParamLutFromCsv(const std::string &csvPath, ImagingParamLUT &lut)
    {
        std::ifstream file(csvPath);
        if (!file.is_open())
        {
            std::cout << "LoadImagingParamLutFromCsv - unable to open " << csvPath << std::endl;
            return false;
        }

        size_t populated = 0;
        std::string line;
        while (std::getline(file, line))
        {
            std::string trimmed = Trim(line);
            if (trimmed.empty() || trimmed[0] == '#')
            {
                continue;
            }

            size_t distanceIdx = 0;
            size_t sparsityIdx = 0;
            ImagingAlgorithmParams parsedParams;
            if (!ParseCsvRow(trimmed, distanceIdx, sparsityIdx, parsedParams))
            {
                std::cout << "LoadImagingParamLutFromCsv - skip line: " << trimmed << std::endl;
                continue;
            }

            if (distanceIdx >= kDistanceBinCount || sparsityIdx >= kSparsityLevelCount)
            {
                std::cout << "LoadImagingParamLutFromCsv - index out of range: " << trimmed << std::endl;
                continue;
            }

            lut[distanceIdx][sparsityIdx] = parsedParams;
            ++populated;
        }

        std::cout << "LoadImagingParamLutFromCsv - populated entries: " << populated << std::endl;
        return populated > 0;
    }

    constexpr std::array<DistanceProfile, kDistanceBinCount> kDistanceProfiles = {{
        {32, 1, 80.0f, 2.0, 8, 3},
        {48, 1, 110.0f, 2.2, 10, 3},
        {64, 2, 140.0f, 2.6, 12, 3},
        {80, 2, 170.0f, 3.0, 14, 5},
        {96, 2, 200.0f, 3.4, 16, 5},
        {120, 3, 230.0f, 3.8, 18, 7},
        {150, 3, 260.0f, 4.2, 20, 7},
    }};

    constexpr std::array<SparsityProfile, kSparsityLevelCount> kSparsityProfiles = {{
        {40, 1, 40.0f, 0.8, -2, 2},
        {20, 1, 20.0f, 0.5, -1, 1},
        {0, 0, 0.0f, 0.0, 0, 0},
        {-10, 0, -15.0f, -0.2, 1, 0},
        {-20, -1, -30.0f, -0.4, 2, -2},
    }};

    ImagingAlgorithmParams ComposeParams(const DistanceProfile &distanceProfile,
                                         const SparsityProfile &sparsityProfile)
    {
        ImagingAlgorithmParams params;
        params.tofFrameCount = std::clamp(distanceProfile.tofFrameCount + sparsityProfile.tofFrameDelta, 16, 256);
        params.reconstructionStride = std::clamp(distanceProfile.reconstructionStride + sparsityProfile.strideDelta, 1, 4);
        params.reconstructionThreshold = std::clamp(distanceProfile.reconstructionThreshold + sparsityProfile.thresholdDelta, 50.0f, 400.0f);
        params.dbscanEps = std::max(0.5, distanceProfile.dbscanEps + sparsityProfile.epsDelta);
        params.dbscanMinSamples = std::clamp(distanceProfile.dbscanMinSamples + sparsityProfile.minSampleDelta, 3, 64);
        int kernel = std::clamp(distanceProfile.kernelSize + sparsityProfile.kernelDelta, 3, 9);
        if ((kernel % 2) == 0)
        {
            ++kernel;
        }
        params.completionKernelSize = kernel;
        return params;
    }

    ImagingParamLUT BuildImagingParamLut()
    {
        ImagingParamLUT lut{};
        for (size_t d = 0; d < kDistanceBinCount; ++d)
        {
            for (size_t s = 0; s < kSparsityLevelCount; ++s)
            {
                lut[d][s] = ComposeParams(kDistanceProfiles[d], kSparsityProfiles[s]);
            }
        }
        if (!LoadImagingParamLutFromCsv(kImagingParamCsvPath, lut))
        {
            std::cout << "BuildImagingParamLut - falling back to baked-in LUT" << std::endl;
        }
        return lut;
    }
} // namespace

const ImagingParamLUT &ImagingParamLut()
{
    static const ImagingParamLUT lut = BuildImagingParamLut();
    return lut;
}

ImagingAlgorithmParams MakeDefaultImagingParams()
{
    return ImagingParamLut()[0][kSparsityLevelCount - 1];
}

size_t ResolveDistanceIndex(float distance)
{
    if (distance < 0.0f)
    {
        distance = 0.0f;
    }
    if (distance >= kMaxDistanceMeters)
    {
        return kDistanceBinCount - 1;
    }
    return static_cast<size_t>(distance / kDistanceBinSizeMeters);
}

size_t ResolveSparsityIndex(float occupancyRatio)
{
    if (occupancyRatio < 0.0f)
    {
        occupancyRatio = 0.0f;
    }
    if (occupancyRatio >= 1.0f)
    {
        return kSparsityLevelCount - 1;
    }
    return static_cast<size_t>(occupancyRatio / kSparsityStep);
}
