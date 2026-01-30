/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-01-27 09:00:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-27 09:09:06
 * @FilePath: /GMAPD_RK3588/task_module/header/udp_send.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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