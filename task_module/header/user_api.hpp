#ifndef __USER_API_H_
#define __USER_API_H_

#include <opencv4/opencv2/opencv.hpp>
#include "apd_reg.h"

int SysInit(void);

int PcieOpen(int id, unsigned int flag);

int PcieRead(int fd, cv::Mat & src);
int PcieRead(int fd, unsigned char * src, int frame_num);

int apd_test_en(int en); // 启用测试模式，en为1表示启用，0表示禁用

int PcieWrite(int fd, uint16_t *depth, uint16_t *intensity);

int fpga_ctl_write(uint32_t offset, uint32_t value);
int fpga_ctl_read(uint32_t offset);

double get_data_frequency(int chl);

#endif