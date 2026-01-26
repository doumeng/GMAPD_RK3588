
#include "uart_read.h"
#include <cstring>

UartCmdType parseUartCommand(
    const char *buf, size_t len,
    SystemConfig &sysCfg,
    HistConfig &histCfg,
    MotionData &motionData)
{
    if (!buf || len < 16)
        return UartCmdType::NONE;

    char cmd = buf[0];

    // 参数配置指令
    if (cmd == 0xC4)
    { 
        {
            char byte12 = buf[1];
            // 工作模式
            sysCfg.workMode = (byte12 & 0x80) ? WorkMode::TEST : WorkMode::STANDARD;
            // 触发方式
            sysCfg.triggerMode = (byte12 & 0x40) ? TriggerMode::INTERNAL : TriggerMode::EXTERNAL;
            // 时间分辨率
            sysCfg.timeResolution = static_cast<float>(byte12 & 0x1F); // 1-4ns
            if (sysCfg.timeResolution < 1.0f)
                sysCfg.timeResolution = 1.0f;

            // 电压
            uint8_t volt_int = buf[2];
            uint8_t volt_dec = buf[3];
            sysCfg.biasVoltage = static_cast<int>(volt_int) + static_cast<int>(volt_dec) / 10.0f;

            // 开门延迟时间
            sysCfg.enDelay = (static_cast<int>(buf[5]) << 8) | buf[4];
        }
        
        {
            // stride 必须大于等于3；threhold必须大于等于2
            uint8_t byte17 = buf[6];
            histCfg.stride = byte17 & 0x0F;
            histCfg.threshold = (byte17 >> 4) & 0x0F;

            if (histCfg.stride < 3) histCfg.stride = 3;
            if (histCfg.threshold < 2) histCfg.threshold = 2;
        }

        return UartCmdType::PARAM_CONFIG;
    }
    // 弹体信息获取指令
    else if (cmd == 0xC6)
    {     
        { 
            // 距离（第12-13字节，单位：米，假设为uint16_t，精度1m）    
            uint16_t dist_raw = (static_cast<uint16_t>(buf[2]) << 8) | buf[1];
            motionData.distance = dist_raw / 1.0f;
            // 速度（第14-15字节，单位：m/s，假设为uint16_t，精度1m/s）
            uint16_t vel_raw = (static_cast<uint16_t>(buf[4]) << 8) | buf[3];
            motionData.velocity = vel_raw / 1.0f;
        }
        return UartCmdType::MOTION_INFO;
    }
    // APD时序启动指令
    else if (cmd == 0xC7)
    { 
        return UartCmdType::APD_Timing_START;
    }
    // 待机指令
    else if (cmd == 0xCA)
    { 
        return UartCmdType::STANDBY;
    }
    // 下电指令
    else if (cmd == 0xCB)
    { 
        return UartCmdType::APD_STOP;
    }

    return UartCmdType::NONE;
}