#ifndef __USER_API_H_
#define __USER_API_H_

#include <opencv4/opencv2/opencv.hpp>
#include "apd_reg.h"

// #ifdef __cplusplus
// extern "C" {
// #endif

int SysInit(void);

int PcieOpen(int id, unsigned int flag);

int PcieRead(int fd, cv::Mat & src);
int PcieRead(int fd, unsigned char * src, int frame_num);

int apd_control_data_send(int control, int sync_delay, int sync_pulse_width,
                          int reset_delay, int reset_pulse_width,
                          int en_delay, int en_pulse_width,
                          int rec_delay, int rec_pulse_width, int test_pulse_width);

// int control 控制指令
// int sync_delay 输出同步延迟（内触发）
// int sync_pulse_width 输出同步脉冲宽度（内触发）
// int reset_delay 复位延迟（）
// int reset_pulse_width 复位脉冲宽度（）
// int en_delay 使能延迟 （控制延迟开门时间）
// int en_pulse_width 使能脉冲宽度
// int rec_delay 接收延迟 （接近en）
// int rec_pulse_width 接收脉冲宽度（）
// int test_pulse_width 测试脉冲宽度（内触发）

int apd_test_en(int en); // 启用测试模式，en为1表示启用，0表示禁用

int PcieWrite(int fd, uint16_t *depth, uint16_t *intensity);

int fpga_ctl_write(uint32_t offset, uint32_t value);
int fpga_ctl_read(uint32_t offset);
double get_data_frequency(int chl);

#if 0
int VideoEncoderOpen(int maxbps, int fps, int gap, int img_width, int img_heigth, int src_pix_format );
int VideoEncoderAndPush(int fd, unsigned char * src);

int send_to_client(unsigned char * buffer, int len);
int recv_from_client(unsigned char * buffer, int len);
int server_socket_init(void);
#endif

// #ifdef __cplusplus
// }
// #endif

#endif