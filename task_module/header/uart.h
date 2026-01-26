/*
 * @Author: 赛狄科技
 * @Date: 2025-05-30 19:52:42
 * @LastEditors: 赛狄科技
 * @LastEditTime: 2025-05-30 19:53:36
 * @FilePath: uart.h
 * @Description: 赛狄串口读取头文件
 */
#ifndef __UART_H__
#define __UART_H__

#include  "serial.h"

#define UARTA "/dev/ttyS4"
#define UARTB "/dev/ttyS5"
#define UARTC "/dev/ttyS6"
#define UARTD "/dev/ttyS7"
#define UARTE "/dev/ttyS9"

// typedef enum serial_parity
// {
//     PARITY_NONE,
//     PARITY_ODD,
//     PARITY_EVEN,
// } serial_parity_t;

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif

/*****************************************************************************************
 * @brief uart_open 打开串口
 *
 * @param  name    设备名，限定UARTB, UARTC ,UARTD
 * @param  baudrate 波特率，持的波特率为 9600 的倍数，如 19200，38400，115200,2000000
 * @return  -1， 初始化失败； 其他，串口索引号index，而非文件描述符fd
 *****************************************************************************************/
int uart_open(int uartchl, uint32_t baudrate, serial_parity_t parity); //step 1
int uart_index2fd(int index);
int uart_fd2index(int fd);
int uart_read(int index, char *buf, int len);
int uart_write(int index, char *buf, int len);
void uart_close(int index);
int uart_start(void);   //step 2

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif