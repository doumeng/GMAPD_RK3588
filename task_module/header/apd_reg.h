#ifndef __APD_REG_
#define __APD_REG_

#include <stdint.h>

#define APD_BASE_ADDR_OFFSET                               0x00010000
#define APD_GATHER_EN_ADDR                                  (APD_BASE_ADDR_OFFSET + 0x0000)
#define APD_CONTORL_ADDR                                       (APD_BASE_ADDR_OFFSET + 0x0004)
#define APD_SYNC_DELAY_ADDR                                 (APD_BASE_ADDR_OFFSET + 0x0008)
#define APD_SYNC_PULSE_WIDTH_ADDR                (APD_BASE_ADDR_OFFSET + 0x000C)
#define APD_RESET_DELAY_ADDR                               (APD_BASE_ADDR_OFFSET + 0x0010)
#define APD_RESET_PULSE_WIDTH_ADDR              (APD_BASE_ADDR_OFFSET + 0x0014)
#define APD_EN_DELAY_ADDR                                       (APD_BASE_ADDR_OFFSET + 0x0018)
#define APD_EN_PULSE_WIDTH_ADDR                      (APD_BASE_ADDR_OFFSET + 0x001C)
#define APD_REC_DELAY_ADDR                                     (APD_BASE_ADDR_OFFSET + 0x0020)
#define APD_REC_PULSE_WIDTH_ADDR                    (APD_BASE_ADDR_OFFSET + 0x0024)
#define APD_TEST_EN_ADDR                                          (APD_BASE_ADDR_OFFSET + 0x0028)
#define APD_TEST_PULSE_WIDTH_ADDR                  (APD_BASE_ADDR_OFFSET + 0x002C)


typedef struct apd_control{
    uint32_t control ;
    uint32_t sync_delay;
    uint32_t sync_pulse_width;
    uint32_t reset_delay;
    uint32_t reset_pulse_width;
    uint32_t en_delay;
    uint32_t en_pulse_width;
    uint32_t rec_delay;
    uint32_t rec_pulse_width;
} apd_control_t;

#endif