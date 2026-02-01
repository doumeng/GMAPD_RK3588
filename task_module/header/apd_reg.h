#ifndef __APD_REG_
#define __APD_REG_

#include <stdint.h>

// 寄存器地址定义
#define APD_BASE_ADDR_OFFSET                    0x00010000
#define APD_GATHER_EN_ADDR                      (APD_BASE_ADDR_OFFSET + 0x0000)
#define APD_CONTORL_ADDR                        (APD_BASE_ADDR_OFFSET + 0x0004)
#define APD_SYNC_DELAY_ADDR                     (APD_BASE_ADDR_OFFSET + 0x0008)
#define APD_SYNC_PULSE_WIDTH_ADDR               (APD_BASE_ADDR_OFFSET + 0x000C)
#define APD_RESET_DELAY_ADDR                    (APD_BASE_ADDR_OFFSET + 0x0010)
#define APD_RESET_PULSE_WIDTH_ADDR              (APD_BASE_ADDR_OFFSET + 0x0014)
#define APD_EN_DELAY_ADDR                       (APD_BASE_ADDR_OFFSET + 0x0018)
#define APD_EN_PULSE_WIDTH_ADDR                 (APD_BASE_ADDR_OFFSET + 0x001C)
#define APD_REC_DELAY_ADDR                      (APD_BASE_ADDR_OFFSET + 0x0020)
#define APD_REC_PULSE_WIDTH_ADDR                (APD_BASE_ADDR_OFFSET + 0x0024)

#define APD_TEST_EN_ADDR                        (APD_BASE_ADDR_OFFSET + 0x0028)
#define APD_TEST_PULSE_WIDTH_ADDR               (APD_BASE_ADDR_OFFSET + 0x002C)
#define APD_CYCLE_CTRL_ADDR                     (APD_BASE_ADDR_OFFSET + 0x0030)
#define APD_TRIGGER_MODE_ADDR                   (APD_BASE_ADDR_OFFSET + 0x0034)
#define APD_DIFF_THREHOLD_ADDR                  (APD_BASE_ADDR_OFFSET + 0x0100)
#define APD_STRIDE_LENGTH_ADDR                  (APD_BASE_ADDR_OFFSET + 0x0104)
#define APD_CONSTRUCT_FRAMES_ADDR               (APD_BASE_ADDR_OFFSET + 0x0200)

#define OTHER_BASE_ADDR_OFFSET                  0x00000000
#define FPGA_VERSION_ADDR                       (OTHER_BASE_ADDR_OFFSET + 0x000C)
#define PCIE_CHL_CTRL_ADDR                      (OTHER_BASE_ADDR_OFFSET + 0x1020)

#endif