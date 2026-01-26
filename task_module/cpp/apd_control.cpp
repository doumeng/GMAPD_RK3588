#include "apd_control.h"

// APD变量偏移地址 10000
// 时序控制
int ApdControl(int control, int sync_delay, int sync_pulse_width,
               int reset_delay, int reset_pulse_width,
               int en_delay, int en_pulse_width,
               int rec_delay, int rec_pulse_width, int test_pulse_width)
{
    if (fpga_ctl_write(APD_CONTORL_ADDR, control) < 0 ||
        fpga_ctl_write(APD_SYNC_DELAY_ADDR, sync_delay) < 0 ||
        fpga_ctl_write(APD_SYNC_PULSE_WIDTH_ADDR, sync_pulse_width) < 0 ||
        fpga_ctl_write(APD_RESET_DELAY_ADDR, reset_delay) < 0 ||
        fpga_ctl_write(APD_RESET_PULSE_WIDTH_ADDR, reset_pulse_width) < 0 ||
        fpga_ctl_write(APD_EN_DELAY_ADDR, en_delay) < 0 ||
        fpga_ctl_write(APD_EN_PULSE_WIDTH_ADDR, en_pulse_width) < 0 ||
        fpga_ctl_write(APD_REC_DELAY_ADDR, rec_delay) < 0 ||
        fpga_ctl_write(APD_REC_PULSE_WIDTH_ADDR, rec_pulse_width) < 0 ||
        fpga_ctl_write(APD_TEST_PULSE_WIDTH_ADDR, test_pulse_width) < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int ApdTestEn(int test_en)
{
    return fpga_ctl_write(APD_TEST_EN_ADDR, test_en);
}

int ApdGatherEn(int gather_en)
{
    return fpga_ctl_write(APD_GATHER_EN_ADDR, gather_en);
}

int ApdGatherEnStatus(){
    return fpga_ctl_read(APD_GATHER_EN_ADDR);
}

int GetFpgaVersion(){
    return fpga_ctl_read(0x000C);
}


// 片内变量 00000
int PcieChlCtrl(int chl)
{
    // chl为0表示原始数据通道
    // chl为1表示点云数据通道
    if (chl < 0 || chl > 1)
    {
        return -1; // 无效通道
    }
    // 通过FPGA控制寄存器设置通道
    return fpga_ctl_write(0x1020, chl);
}

int EnDelayCtrl(int delay)
{
    // 延迟控制寄存器地址为0x1018，delay为延迟开门的计数值
    if (delay < 0)
    {
        return -1; // 无效延迟值
    }
    return fpga_ctl_write(0x10018, delay);
}

int GetEnDelay()
{
    return fpga_ctl_read(0x10018);
}

int RecDelayCtrl(int delay)
{
    // 延迟控制寄存器地址为0x1018，delay为延迟开门的计数值
    if (delay < 0)
    {
        return -1; // 无效延迟值
    }
    return fpga_ctl_write(0x10020, delay);
}

int GetRecDelay()
{
    return fpga_ctl_read(0x10020);
}

int CycleCtrl(int cycle)
{
    if (cycle < 0)
    {
        return -1; // 无效延迟值
    }
    return fpga_ctl_write(0x10030, cycle);
}

int TriggerModeCtrl(int mode)
{
    // 触发模式控制寄存器地址为0x1030，mode为0表示内触发，1表示外触发
    if (mode < 0 || mode > 1)
    {
        return -1; // 无效触发模式
    }
    return fpga_ctl_write(0x10034, mode);
}

int DiffThreholdCtrl(int threhold)
{
    // 差分阈值控制接口，值在0-100
    if (threhold < 0 || threhold > 100)
    {
        return -1; // 无效触发模式
    }
    return fpga_ctl_write(0x10100, threhold);
}

int GetDiffThrehold()
{
    return fpga_ctl_read(0x10100);
}

int StrideLengthCtrl(int length)
{
    // 步长控制接口，值为8 16 32，输入为log2
    if (length < 0 || length > 5)
    {
        return -1; // 无效触发模式
    }
    return fpga_ctl_write(0x10104, length);
}

int GetStrideLength()
{
    return fpga_ctl_read(0x10104);
}

int GateControl(int gate)
{
    if (gate < 0 || gate > 8000)
    {
        return -1; // 无效触发模式
    }

    int end_gate = gate + 1200;

    if (end_gate < 0 || end_gate > 8000)
    {
        return -1; // 无效触发模式
    }

    fpga_ctl_write(0x10108, gate);
    fpga_ctl_write(0x1010C, end_gate);
    
    return 0;
}

int GetStartGate(){
    return fpga_ctl_read(0x10108);
}

int GetEndGate(){
    return fpga_ctl_read(0x1010C);
}
