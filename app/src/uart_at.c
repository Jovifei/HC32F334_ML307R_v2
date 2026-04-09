#include "uart_at.h"
#include "main.h"
#include "config.h"
#include "hc32_ll.h"
#include <string.h>
#include <stdio.h>

// ==================== 静态变量 ====================

static uint8_t s_rx_buf[UART_AT_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;  // 写指针（中断写入）
static uint16_t s_rx_tail = 0;          // 读指针（任务读取）

// 命令结果: 0=等待中, 1=OK, -1=ERROR, -1=超时
static volatile int8_t s_cmd_result = 0;

// URC 回调表
typedef struct {
    char keyword[24];
    urc_callback_t cb;
    bool used;
} urc_entry_t;
static urc_entry_t s_urc_tbl[8];

// ==================== 内部函数 ====================

// 前向声明（ISR 在函数定义之前调用 parse_rx_lines）
static void parse_rx_lines(void);

// USART2 中断服务程序
void USART2_IRQHandler(void)
{
    // RX 中断（每个字节到达时触发）
    if (USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL) == SET) {
        uint16_t data = USART_ReadData(CM_USART2);
        uint8_t ch = (uint8_t)(data & 0xFF);
        USART_ClearStatus(CM_USART2, USART_FLAG_RX_FULL);
        uint16_t next = (s_rx_head + 1) % UART_AT_RX_BUF_SIZE;
        if (next != s_rx_tail) {
            s_rx_buf[s_rx_head] = ch;
            s_rx_head = next;
        }
        (void)USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL);
    }

    // RX timeout 中断（一帧数据接收完毕，RX线空闲10bits时触发）
    if (USART_GetStatus(CM_USART2, USART_FLAG_RX_TIMEOUT) == SET) {
        USART_ClearStatus(CM_USART2, USART_FLAG_RX_TIMEOUT);
        parse_rx_lines();
    }
}

// 从环形缓冲区解析行
static void parse_rx_lines(void)
{
    char line[256];
    uint16_t i = 0;
    bool in_line = false;

    while (s_rx_tail != s_rx_head) {
        uint8_t ch = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;

        if (ch == '\r' || ch == '\n') {
            if (in_line && i > 0) {
                line[i] = '\0';
                in_line = false;

                if (s_cmd_result == 0) {
                    if (strstr(line, "OK") != NULL || strcmp(line, "OK") == 0) {
                        s_cmd_result = 1;
                    } else if (strstr(line, "ERROR") != NULL ||
                               strstr(line, "+CME ERROR") != NULL ||
                               strstr(line, "+CMS ERROR") != NULL) {
                        s_cmd_result = -1;
                    }
                }

                // 检查 URC
                for (int j = 0; j < 8; j++) {
                    if (s_urc_tbl[j].used && strstr(line, s_urc_tbl[j].keyword) != NULL) {
                        if (s_urc_tbl[j].cb) {
                            s_urc_tbl[j].cb(line);
                        }
                        break;
                    }
                }
                i = 0;
            }
        } else if (i < sizeof(line) - 1) {
            line[i++] = (char)ch;
            in_line = true;
        }
    }
}

// ==================== 公开 API ====================

void uart_at_init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
    s_cmd_result = 0;
    memset(s_urc_tbl, 0, sizeof(s_urc_tbl));

    // 使能 USART2 RX 中断（在 NVIC 中已配置优先级）
    USART_FuncCmd(CM_USART2, USART_INT_RX, ENABLE);
}

int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len)
{
    (void)expected_ok;
    if (cmd == NULL) return -2;

    at_flush_rx();

    // 发送 AT 命令（自动加 \r\n）
    char cmd_buf[320];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    uint16_t len = (uint16_t)strlen(cmd_buf);
    for (uint16_t i = 0; i < len; i++) {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET);
        USART_WriteData(CM_USART2, (uint16_t)cmd_buf[i]);
    }

    // 等待响应（轮询方式）
    s_cmd_result = 0;
    uint32_t start = sys_param.timer.timer_1ms_count;

    while (s_cmd_result == 0) {
        parse_rx_lines();
        if ((uint32_t)(sys_param.timer.timer_1ms_count - start) >= timeout_ms) {
            s_cmd_result = -1; // 超时
        }
    }

    // 复制响应
    if (response != NULL && resp_len > 0) {
        uint16_t copied = 0;
        while (s_rx_tail != s_rx_head && copied < (uint16_t)(resp_len - 1)) {
            response[copied++] = s_rx_buf[s_rx_tail];
            s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        }
        response[copied] = '\0';
    }

    return (s_cmd_result == 1) ? 0 : -2;
}

void at_send_raw(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return;
    for (uint16_t i = 0; i < len; i++) {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET);
        USART_WriteData(CM_USART2, (uint16_t)data[i]);
    }
}

void at_register_urc(const char *keyword, urc_callback_t callback)
{
    if (keyword == NULL || callback == NULL) return;
    for (int i = 0; i < 8; i++) {
        if (!s_urc_tbl[i].used) {
            strncpy(s_urc_tbl[i].keyword, keyword, sizeof(s_urc_tbl[i].keyword) - 1);
            s_urc_tbl[i].keyword[sizeof(s_urc_tbl[i].keyword) - 1] = '\0';
            s_urc_tbl[i].cb = callback;
            s_urc_tbl[i].used = true;
            break;
        }
    }
}

void at_flush_rx(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_rx_head = 0;
    s_rx_tail = 0;
    __set_PRIMASK(primask);
}
