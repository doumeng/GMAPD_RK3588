#ifndef BUFFER_H
#define BUFFER_H

#include <vector>
#include <array>
#include <cstdint>
#include <utility>

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
    void push(const T& item);
    void push(T&& item);
    bool popLatest(T& out);
    bool hasData() const;

private:
    void advance();

    std::array<T, Capacity> m_buffer{};
    size_t m_writeIndex = 0;
    size_t m_count = 0;
};

// Implementation
template <typename T, size_t Capacity>
void LatestRingBuffer<T, Capacity>::push(const T& item) {
    m_buffer[m_writeIndex] = item;
    advance();
}

template <typename T, size_t Capacity>
void LatestRingBuffer<T, Capacity>::push(T&& item) {
    m_buffer[m_writeIndex] = std::move(item);
    advance();
}

template <typename T, size_t Capacity>
bool LatestRingBuffer<T, Capacity>::popLatest(T& out) {
    if (m_count == 0) {
        return false;
    }
    const size_t latestIndex = (m_writeIndex + Capacity - 1) % Capacity;
    out = m_buffer[latestIndex];
    m_count = 0;
    return true;
}

template <typename T, size_t Capacity>
bool LatestRingBuffer<T, Capacity>::hasData() const {
    return m_count > 0;
}

template <typename T, size_t Capacity>
void LatestRingBuffer<T, Capacity>::advance() {
    m_writeIndex = (m_writeIndex + 1) % Capacity;
    if (m_count < Capacity) {
        ++m_count;
    }
}

#endif // BUFFER_H

