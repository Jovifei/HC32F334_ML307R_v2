#include "uart_at.h"
#include "config.h"
#include "hc32_ll.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// SysTick 计数器（�? hc32_ll_utility.c 提供，不依赖 ADC 中断�?
extern uint32_t SysTick_GetTick(void);

#define UART_AT_RX_BUF_SIZE 256

// ==================== 静态变�? ====================

static uint8_t s_rx_buf[256];
static volatile uint16_t s_rx_head =
    0;                         // 写指针（中断写入�???
static uint16_t s_rx_tail = 0; // 读指针（任务读取�???

// 命令结果: 0=等待�???, 1=OK, -1=ERROR, -1=超时
static volatile int8_t s_cmd_result = 0;

// RX数据就绪标志（中断设置，主循环清除）
static volatile bool s_rx_data_ready = false;

// 提示符标志：parse_rx_lines_budget 遇到 > 时设置，供证书写入状态机检�?
static volatile bool s_got_prompt = false;

// 写入完成标志：handle_parsed_line 处理�? OK/ERROR 时设�?
static volatile bool s_got_ok  = false;
static volatile bool s_got_err = false;

// RX line assembly buffer (foreground parsing)
static char s_line_buf[256];
static uint16_t s_line_len = 0;

// URC 回调�???
typedef struct
{
    char keyword[24];
    urc_callback_t cb;
    bool used;
} urc_entry_t;
static urc_entry_t s_urc_tbl[4];

// 错误码追踪（AT命令返回CME/CMS ERROR时填充）
static int s_last_error_code = -1;
static char s_last_error_line[32] = {0};

// ==================== 内部函数 ====================

// 前向声明（ISR 在函数定义之前调�??? parse_rx_lines�???
static void parse_rx_lines_budget(uint16_t max_bytes);

static void handle_parsed_line(const char *line)
{
    if (line == NULL || line[0] == '\0')
    {
        return;
    }

    // 调试：打印收到的每一�?
    DEBUG_4G_PRINTF("[UART_AT] RX: [%s]\r\n", line);

    if (s_cmd_result == 0)
    {
        if (strstr(line, "OK") != NULL || strcmp(line, "OK") == 0)
        {
            s_cmd_result = 1;
            s_got_ok = true;
        }
        else if (strstr(line, "+CME ERROR") != NULL || strstr(line, "+CMS ERROR") != NULL ||
                   strstr(line, "ERROR") != NULL)
        {
            s_cmd_result = -1;
            s_got_err = true;
            // 记录错误码（+CME ERROR: N 或 +CMS ERROR: N）
            int err_val = -1;
            if (sscanf(line, "+CME ERROR: %d", &err_val) == 1 ||
                sscanf(line, "+CMS ERROR: %d", &err_val) == 1) {
                s_last_error_code = err_val;
            } else {
                s_last_error_code = -1;
            }
            strncpy(s_last_error_line, line, sizeof(s_last_error_line) - 1);
            s_last_error_line[sizeof(s_last_error_line) - 1] = '\0';
        }
    }

    // 检�??? URC
    for (size_t j = 0; j < sizeof(s_urc_tbl) / sizeof(s_urc_tbl[0]); j++)
    {
        if (s_urc_tbl[j].used && strstr(line, s_urc_tbl[j].keyword) != NULL)
        {
            if (s_urc_tbl[j].cb)
            {
                s_urc_tbl[j].cb(line);
            }
            break;
        }
    }
}

// USART2 中断服务程序（ML307R 4G模组�?
void USART2_Handler(void)
{
    if (USART_GetStatus(CM_USART2, USART_FLAG_RX_FULL))
    {
        uint16_t data = USART_ReadData(CM_USART2);
        uint8_t ch = (uint8_t)(data & 0xFF);
        uint16_t next = (s_rx_head + 1) % UART_AT_RX_BUF_SIZE;
        if (next != s_rx_tail)
        {
            s_rx_buf[s_rx_head] = ch;
            s_rx_head = next;
        }
        s_rx_data_ready = true;
    }

    if (USART_GetStatus(CM_USART2, USART_FLAG_RX_TIMEOUT))
    {
        s_rx_data_ready = true;  // 通知前台有数据待处理
    }

    USART_ClearStatus(CM_USART2, USART_FLAG_ALL);
}

// Ԥ��ʽ������ÿ���������? max_bytes ���ֽڣ�������ѭ��һ�γԿյ��¿���
static void parse_rx_lines_budget(uint16_t max_bytes)
{
    uint16_t processed = 0;
    while (s_rx_tail != s_rx_head && processed < max_bytes)
    {
        uint8_t ch = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        processed++;

        if (ch == '\r' || ch == '\n')
        {
            if (s_line_len > 0)
            {
                s_line_buf[s_line_len] = '\0';
                handle_parsed_line(s_line_buf);
                s_line_len = 0;
            }
            continue;
        }

        if (s_line_len < (sizeof(s_line_buf) - 1))
        {
            s_line_buf[s_line_len++] = (char)ch;
            // '>' 作为行首第一个字�? = 模组的输入提示符，立刻标�?
            if (ch == '>' && s_line_len == 1)
            {
                s_got_prompt = true;
            }
        }
        else
        {
            // �й��������ֶ���ֱ���������У��������?
        }
    }
}

// ==================== 公开 API ====================

void uart_at_init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
    s_cmd_result = 0;
    s_rx_data_ready = false;
    s_line_len = 0;
    memset(s_urc_tbl, 0, sizeof(s_urc_tbl));

    // 使能 USART2 RX 中断和超时中�?
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(USART2_IRQn);
    USART_FuncCmd(CM_USART2, USART_INT_RX, ENABLE);
    USART_FuncCmd(CM_USART2, USART_RX_TIMEOUT, ENABLE);
    USART_FuncCmd(CM_USART2, USART_INT_RX_TIMEOUT, ENABLE);
}

int at_send_command(const char *cmd, const char *expected_ok,
                    uint32_t timeout_ms, char *response, int resp_len)
{
    (void)expected_ok;
    if (cmd == NULL)
        return -2;

    at_flush_rx();

    // 发�? AT 命令（自动加 \r\n�???
    char cmd_buf[320];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);

    uint16_t len = (uint16_t)strlen(cmd_buf);
    for (uint16_t i = 0; i < len; i++)
    {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
            ;
        USART_WriteData(CM_USART2, (uint16_t)cmd_buf[i]);
    }
    // 等待 TX 完全发送完�?
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_CPLT) == RESET)
        ;
    // TX发送完成后在主循环打印调试信息（不在中断上下文�?
    // DEBUG_4G_PRINTF(" UART_AT: TX %d bytes complete\r\n", len);

    // 等待响应（改进的非阻塞轮询）
    s_cmd_result = 0;
    uint32_t start = SysTick_GetTick();

    while (s_cmd_result == 0)
    {
        // 处理 RX 数据
        if (s_rx_data_ready || (s_rx_head != s_rx_tail))
        {
            s_rx_data_ready = false;
            parse_rx_lines_budget(64);
        }

        // 超时检查（使用 SysTick 计数器，不依�? ADC 中断�?
        if ((SysTick_GetTick() - start) >= timeout_ms)
        {
            s_cmd_result = -1; // 超时
            break;
        }
    }

    // 复制响应
    if (response != NULL && resp_len > 0)
    {
        uint16_t copied = 0;
        while (s_rx_tail != s_rx_head && copied < (uint16_t)(resp_len - 1))
        {
            response[copied++] = s_rx_buf[s_rx_tail];
            s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        }
        response[copied] = '\0';
    }

    return (s_cmd_result == 1) ? 0 : -2;
}

void at_send_raw(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return;
    for (uint16_t i = 0; i < len; i++)
    {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
            ;
        USART_WriteData(CM_USART2, (uint16_t)data[i]);
    }
}

void at_register_urc(const char *keyword, urc_callback_t callback)
{
    if (keyword == NULL || callback == NULL)
        return;
    for (size_t i = 0; i < sizeof(s_urc_tbl) / sizeof(s_urc_tbl[0]); i++)
    {
        if (!s_urc_tbl[i].used)
        {
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
    s_rx_data_ready = false;
    s_line_len = 0;
    s_cmd_result = 0;
    s_got_prompt = false;
    s_got_ok     = false;
    s_got_err    = false;
    __set_PRIMASK(primask);
}

void uart_at_process(void)
{
    if (s_rx_data_ready || (s_rx_head != s_rx_tail))
    {
        // 解析 RX 数据
        parse_rx_lines_budget(64);

        // 只有缓冲区空了才清标志，否则保留让下次继续处�?
        if (s_rx_head == s_rx_tail)
            s_rx_data_ready = false;
    }
}

// ==================== 非阻�? AT 命令接口 ====================

static at_nb_state_t s_nb_state = AT_NB_IDLE;
static uint32_t s_nb_start = 0;
static uint32_t s_nb_timeout = 0;

int at_command_start(const char *cmd, uint32_t timeout_ms)
{
    if (cmd == NULL)
        return -1;

    if (s_nb_state == AT_NB_WAITING)
        return 0; // 还在等待上一�?

    at_flush_rx();
    char cmd_buf[320];
    snprintf(cmd_buf, sizeof(cmd_buf), "%s\r\n", cmd);
    uint16_t len = (uint16_t)strlen(cmd_buf);
    for (uint16_t i = 0; i < len; i++)
    {
        while (USART_GetStatus(CM_USART2, USART_FLAG_TX_EMPTY) == RESET)
            ;
        USART_WriteData(CM_USART2, (uint16_t)cmd_buf[i]);
    }
    while (USART_GetStatus(CM_USART2, USART_FLAG_TX_CPLT) == RESET)
        ;

    s_nb_state = AT_NB_WAITING;
    s_nb_start = SysTick_GetTick();
    s_nb_timeout = timeout_ms;
    return 0;
}

int at_command_check(void)
{
    if (s_nb_state == AT_NB_IDLE)
        return AT_NB_IDLE;

    // 检查AT响应结果（由 handle_parsed_line 设置�?
    if (s_cmd_result == 1)
    {
        s_nb_state = AT_NB_OK;
        s_cmd_result = 0;
        return AT_NB_OK;
    }
    if (s_cmd_result == -1)
    {
        s_nb_state = AT_NB_ERR;
        s_cmd_result = 0;
        return AT_NB_ERR;
    }

    // 处理RX数据
    if (s_rx_data_ready || s_rx_head != s_rx_tail)
    {
        s_rx_data_ready = false;
        parse_rx_lines_budget(64);
    }

    // 检查超�?
    if ((SysTick_GetTick() - s_nb_start) >= s_nb_timeout)
    {
        s_nb_state = AT_NB_ERR;
        s_cmd_result = 0;
        return AT_NB_ERR;
    }

    return AT_NB_WAITING;
}

int at_read_response(char *buf, int len)
{
    if (buf == NULL || len <= 0)
        return 0;
    int copied = 0;
    while (s_rx_tail != s_rx_head && copied < len - 1)
    {
        uint8_t ch = s_rx_buf[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % UART_AT_RX_BUF_SIZE;
        buf[copied++] = (char)ch;
    }
    buf[copied] = '\0';
    return copied;
}

// 检查RX缓冲区是否包含指定字符串（用于检�?>提示符等�?
// 返回1=包含, 0=不包�?
int at_rx_contains(const char *str)
{
    if (str == NULL || *str == '\0')
        return 0;
    int str_len = (int)strlen(str);

    // 从s_rx_tail到s_rx_head逐字符检�?
    uint16_t idx = s_rx_tail;
    int matched = 0;
    while (idx != s_rx_head)
    {
        if (s_rx_buf[idx] == (uint8_t)str[matched])
        {
            matched++;
            if (matched == str_len)
                return 1; // 完全匹配
        }
        else
        {
            matched = 0;
            if (s_rx_buf[idx] == (uint8_t)str[0])
                matched = 1;
        }
        idx = (idx + 1) % UART_AT_RX_BUF_SIZE;
    }
    return 0;
}

// ==================== 提示�? / 写入完成 检�? API ====================

/**
 * 检查是否收�? > 提示符（自动清除�?
 * �? parse_rx_lines_budget 在检测到 > 为行首字符时设置
 */
bool at_got_prompt(void)
{
    bool ret = s_got_prompt;
    s_got_prompt = false;
    return ret;
}

/**
 * 检查最近一次写入结果（自动清除�?
 * @return 1=OK, -1=ERROR/CME ERROR, 0=尚未收到
 */
int at_check_last_result(void)
{
    if (s_got_ok)  { s_got_ok  = false; return  1; }
    if (s_got_err) { s_got_err = false; return -1; }
    return 0;
}

// ==================== Error tracking API ====================

int at_get_last_error_code(void)
{
    return s_last_error_code;
}

const char *at_get_last_error_line(void)
{
    return s_last_error_line;
}
