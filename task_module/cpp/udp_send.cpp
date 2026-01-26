#include "udp_send.h"
#include "log.h"
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <time.h>


UdpSender::UdpSender(const std::string& ip, uint16_t port) {
    sockfd_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (sockfd_ < 0) {
        std::cerr << "socket creation failed" << std::endl;
    }
    memset(&dest_addr_, 0, sizeof(dest_addr_));
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &dest_addr_.sin_addr) <= 0) {
        std::cerr << "inet_pton failed: Invalid IP address: " << ip << std::endl;
        close(sockfd_);
    }

    // 非阻塞循环绑定，直到成功
    sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = inet_addr("0.0.0.0");
    local_addr.sin_port = htons(9000); // 指定端口

    while (true) {
        int ret = bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr));
        if (ret == 0) {
            std::cerr << "UDP socket bind success (non-blocking)" << std::endl;
            break;
        } else {
            std::cerr << "UDP socket bind failed, retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cerr << "UdpSender initialized: " << ip << ":" << port << std::endl;
}

UdpSender::~UdpSender() {
    if (sockfd_ >= 0) {
        close(sockfd_);
        Logger::instance().info("UdpSender socket closed");
    }
}

uint32_t UdpSender::getTimestamp() {
    // Equivalent to int(time.time()) in Python: seconds since epoch
    return static_cast<uint32_t>(std::time(nullptr));
}

uint8_t UdpSender::calcChecksum(const UdpFrame& frame) {
    // XOR from header to data (header, ctrl, transfer_id, fragment_idx, data)
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&frame);
    size_t len = offsetof(UdpFrame, checksum); // up to but not including checksum
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum ^= ptr[i];
    }
    return sum & 0xFF;
}

bool UdpSender::sendFrame(const UdpFrame& frame) {
    ssize_t sent = sendto(sockfd_, &frame, UDP_FRAME_TOTAL_LEN, 0,
                          (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    if (sent != UDP_FRAME_TOTAL_LEN) {
        Logger::instance().error(("Failed to send UDP frame, sent bytes: " + std::to_string(sent)).c_str());
        Logger::instance().error(("Expected bytes: " + std::to_string(UDP_FRAME_TOTAL_LEN)).c_str());
        Logger::instance().error(("UDP frame dropped, fragment_idx: " + std::to_string(frame.fragment_idx)).c_str());
        return false;
    }
    Logger::instance().debug(("UDP frame sent, fragment_idx: " + std::to_string(frame.fragment_idx)).c_str());
    return true;
}

bool UdpSender::sendData(const uint8_t* data, size_t length) {
    if (!data || length == 0) {
        Logger::instance().debug("sendData called with empty data or zero length");
        return false;
    }

    // 检查端口是否绑定成功
    if (sockfd_ < 0) {
        Logger::instance().debug("UDP socket not initialized or bind failed, cannot send data");
        return false;
    }

    const size_t max_fragments = 128;
    const size_t fragment_size = UDP_FRAME_DATA_LEN;

    size_t total_fragments = (length + fragment_size - 1) / fragment_size;
    if (total_fragments > max_fragments) {
        Logger::instance().debug("Data too large, only sending first 8 fragments");
        total_fragments = max_fragments; // 最多128片
    }

    uint32_t transfer_id = getTimestamp();

    for (size_t idx = 0; idx < total_fragments; ++idx) {
        UdpFrame frame;
        frame.header = UDP_FRAME_HEADER;

        size_t offset = idx * fragment_size;
        size_t frag_len = std::min(fragment_size, length - offset);

        // ctrl: bit15 is_fragment, bit0-12 fragment length
        frame.ctrl = 0;
        if (total_fragments > 1) frame.ctrl |= (1 << 15); // is_fragment
        frame.ctrl |= (frag_len & 0x1FFF); // 13 bits for length

        frame.transfer_id = transfer_id;
        frame.fragment_idx = static_cast<uint8_t>(idx);

        memset(frame.data, 0, fragment_size);
        memcpy(frame.data, data + offset, frag_len);

        frame.checksum = calcChecksum(frame);
        frame.tail = UDP_FRAME_TAIL;

        if (!sendFrame(frame)) {
            Logger::instance().error(("UDP send failed at fragment idx=" + std::to_string(idx) + ", dropping remaining data.").c_str());
            return false; // 只要有一帧发送失败，立即丢弃剩余数据
        }
        sleep(0.001);
    }
    
    return true;
}
