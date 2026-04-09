#ifndef APP_INC_SUB1G_H_
#define APP_INC_SUB1G_H_

#include "hc32_ll.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "main.h"

// ================= 协议常量定义 =================
#define SUB1G_FRAME_HEADER 0xFACE                      // 帧头
#define SUB1G_MIN_FRAME_SIZE 7                         // 最小帧长度（帧头2B + 地址3B + 长度1B + 命令码1B）
#define SUB1G_MAX_FRAME_SIZE (SUB1G_MAX_DATA_SIZE + 8) // 最大帧长度
#define SUB1G_MAX_DATA_SIZE 128                        // 最大数据内容长度

#define DEVICE_LIST_BUFFER_SIZE 200 // 设备列表缓冲区，为了后台可以读取到已绑定列表和待配对列表

// ================= 命令码定义 =================
typedef enum
{
    CMD_INV_PAIR_BROADCAST = 0x01,              // 微逆配对广播
    CMD_CT_BIND = 0x02,                         // CT绑定
    CMD_CT_UNBIND = 0x03,                       // CT解绑
    CMD_CT_BROADCAST_SINGLE_PHASE_POWER = 0x10, // CT广播单相功率
    CMD_CT_BROADCAST_DATE = 0x11,               // CT广播日期
    CMD_CT_BROADCAST_THREE_PHASE_POWER = 0x12,  // CT广播三相功率
    CMD_CT_BROADCAST_PHASE_INV_COUNT = 0x13,    // CT广播各相设备个数
    CMD_CT_BROADCAST_UNBIND_SN = 0x1B,          // CT广播解绑SN
    CMD_CT_SET_ANTIFLOW = 0x20,                 // CT设置防逆流开关
    CMD_CT_SET_POWER_SWITCH = 0x21,             // CT设置发电开关
    CMD_CT_SET_INV_PHASE = 0x22,                // CT设置微逆所在相
    CMD_CT_ENABLE_PHASE_IDENTIFY = 0x23,        // CT开启相序识别
    CMD_CT_SET_CONNECTION_POINT = 0x24,         // CT设置接入点
    CMD_CT_SET_POWER_LIMIT = 0x25,              // CT设置功率限制
    CMD_CT_CLEAR_DATA = 0x26,                   // CT清除设备运行数据
    CMD_CT_DEBUG_MODE = 0x29,                   // CT开启调试模式
    CMD_CT_SUB1G_VERSION = 0x41,                // CT获取SUB1G版本信息
    CMD_CT_SUB1G_RSSI = 0x42,                   // CT获取SUB1G的RSSI
    CMD_CT_SUB1G_RESTART = 0x43,                // SUB1G重启(OTA完成)
    CMD_CT_SUB1G_CHANNEL_REWORD = 0x44,         // CT发送SubG信道
    CMD_CT_SUB1G_CHANNEL_FORCE = 0x45,          // CT设置ubG信道
    CMD_INV_REPORT_NORMAL = 0x50,               // 微逆日常上报
    CMD_INV_REPORT_ALL = 0x51,                  // 微逆上报全部数据
    CMD_INV_REPORT_NORMAL_2 = 0x52,             // 微逆平时上报2
    CMD_INV_REPORT_NORMAL_3 = 0x54,             // 微逆日常上报3
    CMD_INV_REPORT_SET_1 = 0x55,                // 微逆设置上报1: 接入点 + 功率限制 + sub1g版本
    CMD_INV_REPORT_SET_2 = 0x56,                // 微逆设置上报2: MCU版本 + 相序
    CMD_INV_REPORT_SET_3 = 0x57                 // 微逆设置上报3: 型号 + 信道索引
} sub1g_cmd_t;

// ================= 产品型号定义 =================
typedef enum
{
    PRODUCT_MODEL_800W = 1,      // 800W微逆
    PRODUCT_MODEL_UNKNOWN = 0xFF // 其他型号——后面可添加
} product_model_t;

// ================= 数据帧结构体 =================
// SUB1G通用数据帧结构
typedef struct
{
    uint16_t frame_header;                     // 帧头 0xFACE
    uint32_t sub1g_addr;                       // SUB1G地址（目标或来源）
    uint8_t data_length;                       // 数据长度（命令码1B + 内容nB）
    uint8_t command_code;                      // 命令码
    uint8_t data_content[SUB1G_MAX_DATA_SIZE]; // 数据内容
} sub1g_frame_t;

// ================= 通信状态结构体 =================
// 接收状态
typedef struct
{
    bool frame_received;                   // 帧接收完成标志
    uint8_t rx_buffer[COM_RX_BUFFER_SIZE]; // 接收缓冲区
    uint16_t rx_index;                     // 当前接收索引
} sub1g_uart_com_rx_status_t;

// ============ UART1发送队列数据结构 ============
// 单条消息结构
typedef struct
{
    uint8_t data[UART1_TX_MSG_MAX_LEN]; // 消息内容缓冲区
    uint16_t length;                    // 实际数据长度
} uart1_tx_msg_t;

// 环形队列结构
typedef struct
{
    uart1_tx_msg_t buffer[UART1_TX_QUEUE_SIZE]; // 8个消息槽位
    uint8_t write_index;                        // 入队写指针
    uint8_t read_index;                         // 出队读指针
    uint8_t count;                              // 当前待发送的消息数
    bool is_sending;                            // 发送锁
} uart1_tx_queue_t;

// ================= 微逆配对广播数据结构 =================
// 命令码0x01的数据格式
typedef struct
{
    uint8_t product_model; // 产品型号 (1=800W微逆)
    char device_sn[16];    // 设备SN (15字节+1结束符)
} inv_pair_broadcast_t;

// ================= 全局变量声明 =================
extern sub1g_uart_com_rx_status_t sub1g_status;
extern sub1g_frame_t sub1g_rx_frame;

// 临时接收缓冲区（中断中使用）
extern uint8_t uart1_temp_buffer[SUB1G_MAX_FRAME_SIZE];
extern uint16_t uart1_temp_count;
extern uint16_t uart1_expected_length;

// ================= 函数声明 =================

// 初始化函数
void sub1g_init(void);
void sub1g_reboot(void);

// 接收相关函数
void USART1_Handler(void);               // UART1中断服务函数
void sub1g_rx_task(void);                // 接收任务处理（主循环调用）
void sub1g_process_received_frame(void); // 处理接收到的完整帧

// ============ UART1发送队列 ============
bool uart1_tx_queue_push(const uint8_t *data, uint16_t len); // 将消息加入队列
void uart1_tx_queue_process(void);                           // 处理队列（主循环调用）

// ============ SUB1G发送命令函数 ============
void sub1g_send_bind(uint32_t target_addr);   // 发送绑定命令
void sub1g_send_unbind(uint32_t target_addr); // 发送解绑命令
void sub1g_send_broadcast_single_phase_power(float power, uint8_t inv_count, uint32_t report_addr);
void sub1g_send_broadcast_three_phase_power(int16_t power_ct1, int16_t power_ct2, int16_t power_ct3, uint32_t report_addr);
void sub1g_send_broadcast_date(const char *date);
void sub1g_send_broadcast_phase_inv_count(uint8_t count_ct1, uint8_t count_ct2, uint8_t count_ct3);
void sub1g_send_set_antiflow(uint32_t target_addr, bool enable);
void sub1g_send_set_power_switch(uint32_t target_addr, bool enable);
void sub1g_send_set_inv_phase(uint32_t target_addr, uint8_t phase);
void sub1g_send_set_connection_point(uint32_t target_addr, uint8_t connection_point);
void sub1g_send_set_power_limit(uint32_t target_addr, uint16_t power_limit);
void sub1g_send_clear_data(uint32_t target_addr);
void sub1g_send_enable_phase_identify(uint32_t target_addr, uint8_t time, uint16_t power, uint8_t power_interval);
void sub1g_send_debug_mode(uint32_t target_addr);
void sub1g_send_force_channel(void);
void sub1g_send_get_version(void); // 发送获取SUB1G版本命令(0x41)
void sub1g_send_get_rssi(void);    // 发送获取SUB1G的RSSI命令(0x42)
void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr);

// 用户操作函数
bool user_bind_device(uint32_t sub1g_addr);                 // 用户绑定设备
bool user_unbind_device_by_sub1g_addr(uint32_t sub1g_addr); // 用户解绑设备
const char *get_paired_device_list_string(void);

// INV请求配对列表函数
uint8_t inv_request_pair_list_add(uint32_t sub1g_addr, const char *device_sn, uint8_t product_model);
void inv_request_pair_list_remove(uint32_t sub1g_addr);
int8_t inv_request_pair_list_find_by_addr(uint32_t sub1g_addr);
const char *get_inv_request_pair_list_string(void);

// 用户配对列表函数
bool user_pair_list_add(const char *device_sn);
int8_t user_pair_list_find_by_sn(const char *device_sn);
void user_pair_list_remove_by_sn(const char *device_sn);
bool user_unbind_device_by_sn(const char *device_sn);
void sub1g_send_broadcast_unbind_by_sn(const char *device_sn);
const char *get_user_pair_list_string(void);

// 工具函数
uint8_t find_inv_index_by_sub1g_addr(uint32_t sub1g_addr);
void sub1g_bytes_to_float(const uint8_t *bytes, float *value);
void sub1g_float_to_bytes(float value, uint8_t *bytes);

extern bool user_bind_device(uint32_t sub1g_addr);                 // 用户绑定设备
extern bool user_unbind_device_by_sub1g_addr(uint32_t sub1g_addr); // 用户解绑设备
extern const char *get_paired_device_list_string(void);
extern const char *get_inv_request_pair_list_string(void);
extern const char *get_user_pair_list_string(void);
extern void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr);
#endif /* APP_INC_SUB1G_H_ */
