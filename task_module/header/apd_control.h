#pragma once
#include "user_api.hpp"
#include "apd_reg.h"

// 时序控制
int ApdControl(int control, int sync_delay, int sync_pulse_width,
                          int reset_delay, int reset_pulse_width,
                          int en_delay, int en_pulse_width,
                          int rec_delay, int rec_pulse_width, int test_pulse_width);
// 读取FPGA版本号
int GetFpgaVersion();

int ApdGatherEn(int gather_en);

int PcieChlCtrl(int chl); // PCIE通道控制函数，chl为0表示原始数据通道，1表示点云数据通道

int EnDelayCtrl(int delay); // 延迟控制函数，delay为延迟开门的计数值

int RecDelayCtrl(int delay);

int TriggerModeCtrl(int mode); // 触发模式控制函数，0为内触发，1为外触发

int CycleCtrl(int cycle); // 周期控制函数，cycle为周期值

int ApdTestEn(int test_en);

int StrideLengthCtrl(int length);

int DiffThreholdCtrl(int threhold);

int GateControl(int gate);

int ApdGatherEnStatus();

int GetEnDelay();

int GetRecDelay();

int GetDiffThrehold();

int GetStrideLength();

int GetStartGate();

int GetEndGate();
