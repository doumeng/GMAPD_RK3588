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

static bool udpsend = true;
static float computedistance = 0.0f;
static std::atomic<int> idx(0);

// 全局变量定义
SharedData g_sharedData;
SharedMat g_sharedMat;

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
    // 
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
        g_sharedData.dataUpdated = true;
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

// 发送数据结构定义
enum class UdpPacketType {
    RAW_BYTES,
    POINT_CLOUD_PROCESS
};

struct UdpDataPacket {
    UdpPacketType type = UdpPacketType::RAW_BYTES;

    // RAW_BYTES payload
    std::vector<uint8_t> data;

    // POINT_CLOUD_PROCESS payload
    std::vector<float> dist;
    std::vector<uint16_t> inten;
    std::vector<int32_t> raw; 
    int rows;
    int cols;
};

struct PcieDataPacket {
    int channel;
    std::vector<uint16_t> depth;     // 存储深度数据
    std::vector<uint16_t> intensity; // 存储强度数据
};

template <typename T, size_t Capacity>
class LatestRingBuffer {
public:
    void push(const T& item) {
        m_buffer[m_writeIndex] = item;
        advance();
    }

    void push(T&& item) {
        m_buffer[m_writeIndex] = std::move(item);
        advance();
    }

    bool popLatest(T& out) {
        if (m_count == 0) {
            return false;
        }
        const size_t latestIndex = (m_writeIndex + Capacity - 1) % Capacity;
        out = m_buffer[latestIndex];
        m_count = 0;
        return true;
    }

    bool hasData() const {
        return m_count > 0;
    }

private:
    void advance() {
        m_writeIndex = (m_writeIndex + 1) % Capacity;
        if (m_count < Capacity) {
            ++m_count;
        }
    }

    std::array<T, Capacity> m_buffer{};
    size_t m_writeIndex = 0;
    size_t m_count = 0;
};

constexpr size_t kPacketBufferSize = 5;

LatestRingBuffer<UdpDataPacket, kPacketBufferSize> g_udpRing;
std::mutex g_udpMutex;
std::condition_variable g_udpCV;

LatestRingBuffer<PcieDataPacket, kPacketBufferSize> g_pcieRing;
std::mutex g_pcieMutex;
std::condition_variable g_pcieCV;

// 待实现的接口: 定时更新成像参数
void UpdateImagingParametersInterface() {
    // 只有在数据有更新时才重新计算
    // if (g_sharedData.dataUpdated) 
    {
        float currentDist = g_sharedData.distance;
        float currentSnr = g_sharedData.snr;
        
        // TODO: 根据距离和信噪比及其他信息(如直方图分布)更新成像参数
        // 示例规则: 
        // if (currentSnr < 5.0) { IncreaseExposure(); }
        
        // 目前保持原有逻辑
        update_delay(currentDist);
    }
}

// 线程：定时更新参数
void thread_UpdateParams() {
    Logger::instance().info("Thread ParamUpdate - Starting");
    while (!g_shutdown.load()) {
        wait_until_running();
        if (g_shutdown.load()) {
            break;
        }

        // 执行参数更新逻辑
        UpdateImagingParametersInterface();
        
        // 控制更新频率，例如 100ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
        // 等待有数据或者停止信号
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
                
                update_setting(g_motionData.distance);
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

    u_char *src = new u_char[200 * 16384 * 2];
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
            // 测试模式，工作在通道0，输出原始数据
            int status = PcieRead(0, src, 200);

            double dataFrequency = get_data_frequency(0);

            if (status < 0)
            {
                Logger::instance().debug("Thread ComputeDistance - Failed to read raw data from PCIe");
                continue;
            }
            else
            {
                Logger::instance().info(("Thread ComputeDistance - Successfully read raw data from PCIe, data length: " + std::to_string(status)).c_str());
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
                continue; // 没有新数据，跳过处理 
            }
          
            cv::Mat MatToProcess(128, 128, CV_16UC1, memBuffer.data());

            if (!MatToProcess.empty())
            {
                // 计算直方图并获取结果
                HistogramResult result = ComputeHistogram(MatToProcess, 10, 4000);
                computedistance = result.maxPixelValue / 10.0f;

                g_sharedData.distance = computedistance;
                g_sharedData.snr = result.snr;
                g_sharedData.dataUpdated = true;
                Logger::instance().info(("Thread ComputeDistance - Received new matrix, compute new distance:"+ std::to_string(computedistance) + " SNR:" + std::to_string(result.snr)).c_str());
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