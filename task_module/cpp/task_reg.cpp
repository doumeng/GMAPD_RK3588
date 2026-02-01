#include <fstream>
#include <time.h>
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <array>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <thread>
#include <shared_mutex>
#include <utility>
#include <algorithm>
#include <sstream>
#include <string>

#include "log.h"
#include "compute_distance.h"
#include "point_cloud_process.h"
#include "user_api.hpp"
#include "uart_read.h"
#include "udp_send.h"
#include "util.h"
#include "apd_control.h"
#include "task_reg.h"
#include "uart.h"
#include "buffer.h"

namespace
{
constexpr size_t kDistanceBinCount = 7;
constexpr size_t kSparsityLevelCount = 5;
constexpr float kDistanceBinSizeMeters = 500.0f;
constexpr float kMaxDistanceMeters = 3000.0f;
constexpr float kSparsityStep = 0.2f;
constexpr const char *kImagingParamCsvPath = "/userdata/data/imaging_params.csv";

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

using ImagingParamLUT = std::array<std::array<ImagingAlgorithmParams, kSparsityLevelCount>, kDistanceBinCount>;

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
        Logger::instance().error(("ParseCsvRow - invalid value, line: " + line + ", reason: " + ex.what()).c_str());
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
        Logger::instance().info(("LoadImagingParamLutFromCsv - unable to open " + csvPath).c_str());
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
            Logger::instance().debug(("LoadImagingParamLutFromCsv - skip line: " + trimmed).c_str());
            continue;
        }

        if (distanceIdx >= kDistanceBinCount || sparsityIdx >= kSparsityLevelCount)
        {
            Logger::instance().debug(("LoadImagingParamLutFromCsv - index out of range: " + trimmed).c_str());
            continue;
        }

        lut[distanceIdx][sparsityIdx] = parsedParams;
        ++populated;
    }

    Logger::instance().info(("LoadImagingParamLutFromCsv - populated entries: " + std::to_string(populated)).c_str());
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
        Logger::instance().info("BuildImagingParamLut - falling back to baked-in LUT");
    }
    return lut;
}

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

} // namespace

static bool udpsend = true;
static float computedistance = 0.0f;
static std::atomic<int> idx(0);

// 全局变量定义
SharedData g_sharedData;
SharedMat g_sharedMat;

ImagingAlgorithmParams g_imagingParams = MakeDefaultImagingParams();
std::mutex g_imagingParamMutex;

SystemConfig g_sysConfig;
MotionData g_motionData;
HistConfig g_histConfig;

std::atomic<bool> g_stopThreads(false);
std::string g_outputDir = "/userdata/data/"; // 输出目录

// UDP 发送器实例
std::string ip_address = "192.168.20.111";
uint16_t udp_port = 10000;
static UdpSender udp_Sender(ip_address, udp_port);

std::mutex g_stateMutex;
std::condition_variable g_stateCV;
std::atomic<bool> g_shutdown(false);

// 推流环形缓冲区及同步原语
constexpr size_t kPacketBufferSize = 5;

LatestRingBuffer<UdpDataPacket, kPacketBufferSize> g_udpRing;
std::mutex g_udpMutex;
std::condition_variable g_udpCV;

LatestRingBuffer<PcieDataPacket, kPacketBufferSize> g_pcieRing;
std::mutex g_pcieMutex;
std::condition_variable g_pcieCV;


void start_threads()
{
    Logger::instance().info("Starting all threads");
    g_stopThreads = false;
    g_stateCV.notify_all();
    g_udpCV.notify_all();
    g_pcieCV.notify_all();
}

void stop_threads()
{
    Logger::instance().info("Stopping all threads");
    g_stopThreads = true;
    g_udpCV.notify_all();
    g_pcieCV.notify_all();
    g_stateCV.notify_all();
}

static void wait_until_running()
{
    if (!g_stopThreads.load())
    {
        return;
    }

    std::unique_lock<std::mutex> stateLock(g_stateMutex);
    g_stateCV.wait(stateLock, [] { return !g_stopThreads.load() || g_shutdown.load(); });
}

void update_delay(float distance)
{
    // TODO 
    int timedelay = ComputeDelay(distance, 2, 1000); 

    Logger::instance().info(("update_setting - Computation started with distance: " + std::to_string(distance)).c_str());
    Logger::instance().info(("update_setting - Computation started with time delay: " + std::to_string(timedelay)).c_str());
    
    int en_delay = static_cast<int>(timedelay / 5);
    int en_status = EnDelayCtrl(en_delay);
    int rec_status = RecDelayCtrl(en_delay + 1);
    {
        g_sharedData.timedelay = timedelay * 2;
        g_sharedData.distance = g_motionData.distance;
        g_sharedData.velocity = g_motionData.velocity;
    }

    if (en_status > 0 && rec_status > 0)
    {
        Logger::instance().debug("update_setting - en delay setting finished");
    }
    else
    {
        Logger::instance().debug("update_setting - en delay setting failed");
    }

    return ;
}

// 成像参数根据离线 LUT 进行 O(1) 查询
void UpdateImagingParametersInterface()
{
    if (!g_sharedData.dataUpdated)
    {
        return;
    }

    const float currentDistance = g_sharedData.distance;
    const float occupancyRatio = g_sharedData.occupancyRatio;

    const size_t distanceIdx = ResolveDistanceIndex(currentDistance);
    const size_t sparsityIdx = ResolveSparsityIndex(occupancyRatio);
    const ImagingAlgorithmParams paramsFromLut = ImagingParamLut()[distanceIdx][sparsityIdx];

    {
        std::lock_guard<std::mutex> lock(g_imagingParamMutex);
        g_imagingParams = paramsFromLut;
    }

    g_sharedData.dataUpdated = false;

    Logger::instance().info(("UpdateImagingParametersInterface - distanceIdx: " + std::to_string(distanceIdx) +
                             ", sparsityIdx: " + std::to_string(sparsityIdx) +
                             ", frames: " + std::to_string(paramsFromLut.tofFrameCount) +
                             ", stride: " + std::to_string(paramsFromLut.reconstructionStride) +
                             ", eps: " + std::to_string(paramsFromLut.dbscanEps) +
                             ", kernel: " + std::to_string(paramsFromLut.completionKernelSize)).c_str());
}

// 线程：定时更新参数
void thread_UpdateParams() {
    Logger::instance().info("Thread ParamUpdate - Starting");
    while (!g_shutdown.load()) {
        wait_until_running();
        if (g_shutdown.load()) {
            break;
        }

        UpdateImagingParametersInterface();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    Logger::instance().info("Thread ParamUpdate - Stopped");
}

// 线程：UDP单独发送
void thread_UdpSend() {
    Logger::instance().info("Thread UdpSend - Starting");
    while (!g_shutdown.load()) {
        wait_until_running();
        if (g_shutdown.load()) {
            break;
        }

        std::unique_lock<std::mutex> lock(g_udpMutex);

        if (!g_udpRing.hasData()) {
            g_udpCV.wait_for(lock, std::chrono::milliseconds(100), [&]{
                return g_udpRing.hasData() || g_shutdown.load() || g_stopThreads.load();
            });
        }

        if (g_shutdown.load()) {
            lock.unlock();
            break;
        }

        if (g_stopThreads.load()) {
            lock.unlock();
            continue;
        }

        UdpDataPacket pkt;
        if (!g_udpRing.popLatest(pkt)) {
            lock.unlock();
            continue;
        }
        lock.unlock();

        if (udpsend) {
            if (pkt.type == UdpPacketType::RAW_BYTES && !pkt.data.empty()) {
                udp_Sender.sendData(pkt.data.data(), pkt.data.size());
            }
            else if (pkt.type == UdpPacketType::POINT_CLOUD_PROCESS) {
                size_t pixel_count = pkt.rows * pkt.cols;
                if (pkt.dist.size() == pixel_count && pkt.inten.size() == pixel_count && pkt.raw.size() == pixel_count) {
                    uint32_t * data = interleaveArrays(pkt.dist.data(), pkt.inten.data(), pixel_count);

                    size_t data_size = pixel_count * sizeof(uint32_t);
                    size_t raw_data_size = pixel_count * sizeof(int32_t);
                    size_t total_size = data_size + raw_data_size;

                    std::vector<uint8_t> combined_data(total_size);
                    std::memcpy(combined_data.data(), data, data_size);
                    std::memcpy(combined_data.data() + data_size, pkt.raw.data(), raw_data_size);

                    udp_Sender.sendData(combined_data.data(), total_size);

                    delete[] data;
                }
            }
        }
    }
    Logger::instance().info("Thread UdpSend - Stopped");
}

// 线程：PCIE单独发送
void thread_PcieSend() {
    Logger::instance().info("Thread PcieSend - Starting");
    while (!g_shutdown.load()) {
        wait_until_running();
        if (g_shutdown.load()) {
            break;
        }

        std::unique_lock<std::mutex> lock(g_pcieMutex);
        if (!g_pcieRing.hasData()) {
            g_pcieCV.wait_for(lock, std::chrono::milliseconds(100), [&]{
                return g_pcieRing.hasData() || g_shutdown.load() || g_stopThreads.load();
            });
        }

        if (g_shutdown.load()) {
            lock.unlock();
            break;
        }

        if (g_stopThreads.load()) {
            lock.unlock();
            continue;
        }

        PcieDataPacket pkt;
        if (!g_pcieRing.popLatest(pkt)) {
            lock.unlock();
            continue;
        }
        lock.unlock();

        if (!pkt.depth.empty() && !pkt.intensity.empty()) {
            PcieWrite(pkt.channel, pkt.depth.data(), pkt.intensity.data());
        }
    }
    Logger::instance().info("Thread PcieSend - Stopped");
}

void thread_Communication()
{
    Logger::instance().info("Thread_Communication - Starting communication thread");

    // 初始化UART
    int uart_chl = uart_open(0, 115200, serial_parity_t::PARITY_NONE); // 假设使用UART1，波特率115200，无校验位

    if (uart_chl == -1)
    {
        Logger::instance().debug("Thread_Communication - Failed to open UART");
        return;
    }

    Logger::instance().info("Thread_Communication - UART opened successfully");
    uart_start(); // 启动UART通信

    // 指令获取
    char buf[16];    // 假设最大接收长度为32字节
    size_t len = 16; // 实际接收长度

    // 循环读取uart指令并解析
    while (!g_shutdown.load())
    {
        int read_status = uart_read(0, buf, len);

        if (read_status > 0)
        {
            Logger::instance().info("Thread_Communication - UART data received");

            // 获取并更新参数配置
            UartCmdType cmdType = parseUartCommand(buf, len, g_sysConfig, g_histConfig, g_motionData);

            if (cmdType == UartCmdType::PARAM_CONFIG)
            {
                Logger::instance().info("Thread_Communication - Parameter configuration received");
                // 更新系统配置
                Logger::instance().debug(("Thread_Communication - Work Mode: " + std::to_string(static_cast<int>(g_sysConfig.workMode))).c_str());
                Logger::instance().debug(("Thread_Communication - Trigger Mode: " + std::to_string(static_cast<int>(g_sysConfig.triggerMode))).c_str());
                Logger::instance().debug(("Thread_Communication - Time Resolution: " + std::to_string(g_sysConfig.timeResolution)).c_str());
                Logger::instance().debug(("Thread_Communication - Bias Voltage: " + std::to_string(g_sysConfig.biasVoltage)).c_str());
                Logger::instance().debug(("Thread_Communication - En Delay: " + std::to_string(g_sysConfig.enDelay)).c_str());
                Logger::instance().debug(("Thread_Communication - stride: " + std::to_string(g_histConfig.stride)).c_str());
                Logger::instance().debug(("Thread_Communication - threshold: " + std::to_string(g_histConfig.threshold)).c_str());
                
                // 设置工作模式
                if (g_sysConfig.workMode == WorkMode::TEST)
                {
                    int status = PcieChlCtrl(0);

                    if (status < 0)
                    {
                        Logger::instance().debug("Thread_Communication - channel 0 enable failed");
                    }
                    else
                    {
                        idx = 0;
                        Logger::instance().debug("Thread_Communication - channel 0 enabled");
                    }
                }
                else if (g_sysConfig.workMode == WorkMode::STANDARD)
                {
                    int status = PcieChlCtrl(1);

                    if (status < 0)
                    {
                        Logger::instance().debug("Thread_Communication - channel 1 enable failed");
                    }
                    else
                    {
                        idx = 0;
                        Logger::instance().debug("Thread_Communication - channel 1 enabled");
                    }
                }

                // 设置触发方式
                if (g_sysConfig.triggerMode == TriggerMode::INTERNAL)
                {
                    int status = TriggerModeCtrl(0);
                    
                    if (status < 0)
                    {
                        Logger::instance().debug("Thread_Communication - internal trigger enable failed");
                    }
                    else
                    {
                        Logger::instance().debug("Thread_Communication - internal trigger enabled");
                        
                        int status = CycleCtrl(20000);
                        
                        if (status < 0)
                        {
                            Logger::instance().debug("Thread_Communication - internal trigger cycle setting failed");
                        }
                        else
                        {
                            Logger::instance().debug("Thread_Communication - internal trigger cycle setting finished");
                        }
                    }
                }
                else if (g_sysConfig.triggerMode == TriggerMode::EXTERNAL)
                {
                    int status = TriggerModeCtrl(1);

                    if (status < 0)
                    {
                        Logger::instance().debug("Thread_Communication - external trigger enable failed");
                    }
                    else
                    {
                        Logger::instance().debug("Thread_Communication - external trigger enabled");
                    }
                }
            }
            else if (cmdType == UartCmdType::MOTION_INFO)
            {
                // 更新运动数据
                Logger::instance().info("Thread_Communication - Motion data received");
                Logger::instance().debug(("Thread_Communication - Velocity: " + std::to_string(g_motionData.velocity)).c_str());
                Logger::instance().debug(("Thread_Communication - Distance: " + std::to_string(g_motionData.distance)).c_str());
                Logger::instance().debug(("Thread_Communication - Roll: " + std::to_string(g_motionData.roll)).c_str());
                Logger::instance().debug(("Thread_Communication - Pitch: " + std::to_string(g_motionData.pitch)).c_str());
                Logger::instance().debug(("Thread_Communication - Yaw: " + std::to_string(g_motionData.yaw)).c_str());
                
                update_delay(g_motionData.distance);
            }
            else if (cmdType == UartCmdType::APD_Timing_START) // APD时序启动指令
            {
                ApdGatherEn(1);
                start_threads();
                Logger::instance().info("Thread_Communication - APD START command received, and start apd time squence..");
            }
            else if (cmdType == UartCmdType::STANDBY) // 待机指令
            {
                stop_threads();
                Logger::instance().info("Thread_Communication - APD Standby command received.");
            }
            else if (cmdType == UartCmdType::APD_STOP) // 下电指令
            {
                ApdGatherEn(0); // 停时序
                g_shutdown = true;
                stop_threads();
                Logger::instance().info("Thread_Communication - APD Stop command received, and stop apd time squence.");
                break;
            }
        }
        else
        {
            continue;
        }
    }

    uart_close(uart_chl);
    Logger::instance().info("Thread_Communication - UART closed");
}

void thread_ComputeDistance()
{
    constexpr auto kComputeInterval = std::chrono::milliseconds(50); // 模拟1秒读取一次

    constexpr size_t kMaxTofFrameCount = 256;
    u_char *src = new u_char[kMaxTofFrameCount * 16384 * 2];
    int chl = 0;
    std::vector<uint16_t> memBuffer(128 * 128);

    while (!g_shutdown.load())
    {
        wait_until_running();
        if (g_shutdown.load())
        {
            break;
        }

        auto computationStart = std::chrono::steady_clock::now();

        // 读取原始飞行时间数据

        if (g_sysConfig.workMode == WorkMode::TEST)
        {
            chl = 0;
            int tofFrameCount = 200;
            {
                std::lock_guard<std::mutex> lock(g_imagingParamMutex);
                tofFrameCount = g_imagingParams.tofFrameCount;
            }
            tofFrameCount = std::clamp(tofFrameCount, 1, static_cast<int>(kMaxTofFrameCount));

            // 测试模式，工作在通道0，输出原始数据
            int status = PcieRead(0, src, tofFrameCount);

            double dataFrequency = get_data_frequency(0);

            if (status < 0)
            {
                Logger::instance().debug("Thread ComputeDistance - Failed to read raw data from PCIe");
                continue;
            }
            else
            {
                Logger::instance().info(("Thread ComputeDistance - Successfully read raw data from PCIe, data length: " + std::to_string(status) +
                                         ", frames: " + std::to_string(tofFrameCount)).c_str());
                Logger::instance().info(("Thread ComputeDistance - Tof data frequency: " + std::to_string(dataFrequency)).c_str());

                {
                    UdpDataPacket pkt;
                    size_t dataLen = 4 * 2 * 16384 * sizeof(unsigned char);
                    pkt.data.resize(dataLen);

                    memcpy(pkt.data.data(), src + 2 * 80 * 16384 * sizeof(unsigned char), dataLen);

                    {
                        std::lock_guard<std::mutex> lock(g_udpMutex);
                        g_udpRing.push(std::move(pkt));
                    }
                    g_udpCV.notify_one();
                }
            }
        }
        else if (g_sysConfig.workMode == WorkMode::STANDARD)
        {
            bool hasNewData = false;
            {
                std::unique_lock<std::shared_mutex> lock(g_sharedMat.matMutex);
                if (g_sharedMat.newDataAvailable)
                {
                    memcpy(memBuffer.data(), g_sharedMat.sharedMat, memBuffer.size() * sizeof(uint16_t));
                    g_sharedMat.newDataAvailable = false;
                    hasNewData = true;
                }
            }

            if (!hasNewData)
            {
                continue; 
            }
          
            cv::Mat MatToProcess(128, 128, CV_16UC1, memBuffer.data());

            if (!MatToProcess.empty())
            {
                // 计算直方图并获取结果
                HistogramResult result = ComputeHistogram(MatToProcess, 1000, 50000);
                computedistance = result.maxPixelValue / 10.0f;

                g_sharedData.distance = computedistance;
                g_sharedData.occupancyRatio = result.occupancyRatio;
                g_sharedData.dataUpdated = true;
                Logger::instance().info(("Thread ComputeDistance - Distance: " + std::to_string(computedistance) +
                                         " m, occupancy: " + std::to_string(result.occupancyRatio)).c_str());
            }
        }

        auto computationTime = std::chrono::steady_clock::now() - computationStart;
        long long duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(computationTime).count();
        Logger::instance().debug(("Thread ComputeDistance - Computation time: " + std::to_string(duration_ms) + "ms").c_str());

        if (computationTime < kComputeInterval)
        {
            std::this_thread::sleep_for(kComputeInterval - computationTime);
        }
    }

    delete[] src;
}

void thread_PointCloudProcess()
{
    constexpr auto kComputeInterval = std::chrono::milliseconds(50);

    cv::Mat src = cv::Mat(128, 128, CV_32SC1, cv::Scalar(0));

    float *denoised_distance = createFloatMatrix(src.rows, src.cols);
    uint16_t *denoised_intensity = createUint16Matrix(src.rows, src.cols);

    while (!g_shutdown.load())
    {
        wait_until_running();
        if (g_shutdown.load())
        {
            break;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        // 数据范围0-8191，需用32位单通道Mat
        if (g_sysConfig.workMode == WorkMode::STANDARD)
        {
            // 工作在通道1，线程开始读取点云并降噪
            int status = PcieRead(1, src);

            if (status < 0)
            {
                Logger::instance().debug("Thread PointCloudProcess - Failed to read point cloud from PCIe");
                continue;
            }
            else
            { 
                Logger::instance().info(("Thread PointCloudProcess - Successfully read point cloud from PCIe, data length: " + std::to_string(status)).c_str());
                Logger::instance().info(("Thread PointCloudProcess - " + std::to_string(g_sharedData.timedelay) + " ns delay").c_str());
                
                double dataFrequency = get_data_frequency(1);
                Logger::instance().info(("Thread PointCloudProcess - Depth data frequency: " + std::to_string(dataFrequency)).c_str());

                cv::rotate(src, src, cv::ROTATE_90_COUNTERCLOCKWISE); // 逆时针旋转90度
                cv::flip(src, src, 1); // 水平翻转
                
                PointCloudProcess(g_sharedData.timedelay, src, denoised_distance, denoised_intensity);
                
                uint16_t * int_denoised_distance = float2Uint16(denoised_distance, 128*128);
                {
                    std::lock_guard<std::shared_mutex> lock(g_sharedMat.matMutex);
                    memcpy(g_sharedMat.sharedMat, int_denoised_distance, src.rows*src.cols*sizeof(uint16_t)); // 深拷贝，避免引用悬挂
                    g_sharedMat.newDataAvailable=true;
                }
                
                // 发送 PCIe 数据 (放入队列)
                {
                    PcieDataPacket pciePkt;
                    pciePkt.channel = 1;
                    size_t sz = src.rows * src.cols;
                    pciePkt.depth.assign(int_denoised_distance, int_denoised_distance + sz);
                    pciePkt.intensity.assign(denoised_intensity, denoised_intensity + sz);
                    
                    {
                        std::lock_guard<std::mutex> lock(g_pcieMutex);
                        g_pcieRing.push(std::move(pciePkt));
                    }
                    g_pcieCV.notify_one();
                }

                // 发送 UDP 数据 (放入队列)
                {
                    UdpDataPacket udpPkt;
                    udpPkt.type = UdpPacketType::POINT_CLOUD_PROCESS;
                    udpPkt.rows = src.rows;
                    udpPkt.cols = src.cols;
                    
                    size_t pixel_count = src.rows * src.cols;

                    udpPkt.dist.assign(denoised_distance, denoised_distance + pixel_count);
                    udpPkt.inten.assign(denoised_intensity, denoised_intensity + pixel_count);
                    
                    if (src.isContinuous()) {
                        const int32_t* ptr = src.ptr<int32_t>(0);
                        udpPkt.raw.assign(ptr, ptr + pixel_count);
                    } else {

                         udpPkt.raw.resize(pixel_count);
                         int idx_raw = 0;
                         for(int r=0; r<src.rows; ++r) {
                            const int32_t* rowptr = src.ptr<int32_t>(r);
                            for(int c=0; c<src.cols; ++c) {
                                udpPkt.raw[idx_raw++] = rowptr[c];
                            }
                         }
                    }
                    
                    {
                        std::lock_guard<std::mutex> lock(g_udpMutex);
                        g_udpRing.push(std::move(udpPkt));
                    }
                    g_udpCV.notify_one();
                }
                
                delete[] int_denoised_distance; 

                memset(denoised_distance, 0, src.rows * src.cols * sizeof(float));
                memset(denoised_intensity, 0, src.rows * src.cols * sizeof(uint16_t));
            }
        }
        else if (g_sysConfig.workMode == WorkMode::TEST)
        {
            continue;
        }

        auto computationTime = std::chrono::high_resolution_clock::now() - start_time;
        long long duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(computationTime).count();
        Logger::instance().info(("Thread PointCloudProcess - Computation time: " + std::to_string(duration_ms) + "ms").c_str());

        if (computationTime < kComputeInterval)
        {
            std::this_thread::sleep_for(kComputeInterval - computationTime);
        }
    }
    src.release();
}

void register_threads()
{
    Logger::instance().info("Registering and starting threads");

    ApdControl(1, 2000, 40, 19600, 200, 0, 3200, 1, 3, 1600);
    
    Logger::instance().info("APD control initialized");

    int version = GetFpgaVersion();
    Logger::instance().info(("FPGA Version: " + std::to_string(version)).c_str());

    std::thread parseUart(thread_Communication);
    parseUart.detach();

    std::thread rawoutput(thread_ComputeDistance);
    rawoutput.detach();

    std::thread pcoutput(thread_PointCloudProcess);
    pcoutput.detach();

    std::thread udpSend(thread_UdpSend);
    udpSend.detach();

    std::thread pcieSend(thread_PcieSend);
    pcieSend.detach();
    
    std::thread paramUpdate(thread_UpdateParams);
    paramUpdate.detach();

    while (true)
        std::this_thread::sleep_for(std::chrono::seconds(1));
}