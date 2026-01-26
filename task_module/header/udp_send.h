#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <queue>
#include <mutex>
#include <condition_variable>

#include "log.h"

constexpr uint16_t UDP_FRAME_HEADER = 0xAA55;
constexpr uint16_t UDP_FRAME_TAIL   = 0x55AA;
constexpr size_t   UDP_FRAME_DATA_LEN = 4096;
constexpr size_t   UDP_FRAME_TOTAL_LEN = 4108; // 2+2+4+1+4096+1+2

struct UdpFrame {
    uint16_t header;         // 0xAA55
    uint16_t ctrl;           // bit15: is_fragment, bit0-12: fragment length
    uint32_t transfer_id;    // timestamp or unique id
    uint8_t  fragment_idx;   // fragment index (0-15)
    uint8_t  data[UDP_FRAME_DATA_LEN]; // payload
    uint8_t  checksum;       // XOR of all bytes from header to data
    uint16_t tail;           // 0x55AA
} __attribute__((packed));

struct UdpPacket {
    std::vector<uint8_t> data;
};

struct PciePacket {
    std::vector<uint16_t> dist;
    std::vector<uint16_t> intensity;
};
template<typename T>
class ThreadSafeQueue {
public:
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.push(value);
        cv_.notify_one();
    }
    bool pop(T& value) {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]{ return !queue_.empty(); });
        value = queue_.front();
        queue_.pop();
        return true;
    }
    bool try_pop(T& value) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return false;
        value = queue_.front();
        queue_.pop();
        return true;
    }
private:
    std::queue<T> queue_;
    std::mutex mtx_;
    std::condition_variable cv_;
};


class UdpSender {
public:
    UdpSender(const std::string& ip, uint16_t port);
    ~UdpSender();

    // 发送大数据，自动分片组帧
    bool sendData(const uint8_t* data, size_t length);

private:
    int sockfd_;
    struct sockaddr_in dest_addr_;

    // 组帧并发送单个分片
    bool sendFrame(const UdpFrame& frame);

    // 计算校验
    uint8_t calcChecksum(const UdpFrame& frame);

    // 获取当前时间戳作为传输ID
    uint32_t getTimestamp();
};