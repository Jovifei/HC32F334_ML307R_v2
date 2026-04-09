#ifndef __OTA_MANAGER_H__
#define __OTA_MANAGER_H__

#include "hc32_ll.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "debug.h"

// ================= OTA常量定义 =================
#define OTA_FIRMWARE_BUFFER_SIZE (128)       // 单次固件传输大小（WiFi侧），必须是OTA_SUB1G_PACKET_SIZE的整数倍
#define OTA_SUB1G_PACKET_SIZE (64)           // Sub1G OTA单包大小
#define OTA_RETRY_MAX (10)                   // 单次命令最大重试次数
#define OTA_DEVICE_RETRY_MAX (3)             // 单个设备升级失败后的最大重试次数
#define OTA_FIRMWARE_RETRY_MAX (50)          // 固件级别最大重试次数(部分成功时)
#define OTA_TIMEOUT_MS (500)                 // 单包超时时间(0.5秒)
#define OTA_VERSION_QUERY_TIMEOUT_MS (20000) // 询问版本总超时时间(20秒)
#define OTA_VERSION_QUERY_INTERVAL_MS (200)  // 询问版本间隔时间(200毫秒)
#define OTA_VERSION_MAX_LEN (17)             // 版本号最大长度

// ================= OTA命令码定义（Sub1G协议） =================
#define SUB1G_OTA_CMD_INIT (0x70)            // 初始化升级命令
#define SUB1G_OTA_CMD_INIT_ACK (0x71)        // 初始化应答
#define SUB1G_OTA_CMD_DATA (0x72)            // 固件数据传输
#define SUB1G_OTA_CMD_DATA_ACK (0x73)        // 数据应答
#define SUB1G_OTA_CMD_CANCEL_FROM_CT (0x74)  // CT取消升级
#define SUB1G_OTA_CMD_CANCEL_FROM_DEV (0x75) // 设备取消升级
#define SUB1G_OTA_CMD_QUERY_VERSION (0x76)   // CT询问设备版本
#define SUB1G_OTA_CMD_VERSION_REPORT (0x77)  // 设备上报版本

// ================= WiFi OTA命令码定义 =================
#define WIFI_OTA_CMD_START (1000)   // 升级开始命令
#define WIFI_OTA_CMD_REQUEST (1001) // 请求固件数据
#define WIFI_OTA_CMD_FINISH (1002)  // 升级完成通知

// ================= OTA固件类型定义 =================
typedef enum
{
    FW_TYPE_UNKNOWN = 0,   // 未知类型
    FW_TYPE_CT_SUB1G = 3,  // CT的Sub1G模块固件
    FW_TYPE_INV_SUB1G = 4, // 微逆的Sub1G模块固件
    FW_TYPE_INV_800W = 5,  // 800W微逆MCU固件
    FW_TYPE_INV_2500W = 6  // 2500W微逆MCU固件
} ota_fw_type_t;

// ================= Sub1G OTA设备类型 =================
typedef enum
{
    SUB1G_OTA_TYPE_CT_SUB1G = 0x11,  // CT的Sub1G模块
    SUB1G_OTA_TYPE_INV_SUB1G = 0x12, // 微逆的Sub1G模块
    SUB1G_OTA_TYPE_INV_800W = 0x13,  // 800W微逆MCU
    SUB1G_OTA_TYPE_INV_2500W = 0x14  // 2500W微逆MCU
} sub1g_ota_type_t;

// ================= OTA状态定义 =================
typedef enum
{
    OTA_STATE_IDLE = 0,        // 空闲状态
    OTA_STATE_INIT_WAIT_ACK,   // 等待初始化应答
    OTA_STATE_TRANSMITTING,    // 数据传输中
    OTA_STATE_WAIT_DATA_ACK,   // 等待数据应答
    OTA_STATE_QUERY_VERSION,   // 主动询问微逆版本（升级完成后）
    OTA_STATE_WAIT_CT_VERSION, // 等待CT Sub1G上报版本（升级完成后）
    OTA_STATE_COMPLETED,       // 升级完成
    OTA_STATE_FAILED,          // 升级失败
    OTA_STATE_CANCELLED,       // 升级取消
    OTA_STATE_INIT_WAIT_READY
} ota_state_t;

// ================= OTA升级结果定义 =================
typedef enum
{
    OTA_RESULT_SUCCESS_ALL = 0,     // 全部成功
    OTA_RESULT_PARTIAL_SUCCESS = 1, // 部分成功
    OTA_RESULT_FAILED = 2           // 全部失败
} ota_result_t;

// ================= OTA启动命令返回值定义 =================
typedef enum
{
    OTA_START_READY = 0, // 准备就绪，可以开始升级
    OTA_START_BUSY = 1,  // 繁忙，正在进行其他OTA升级
    OTA_START_ERROR = 2, // 错误，参数无效或无设备可升级
    OTA_START_WAIT = 3   // 错误，版本未统计完全
} ota_start_result_t;

// ================= 单个设备OTA信息结构体 =================
typedef struct
{
    uint32_t sub1g_addr;                     // Sub1G地址
    uint8_t device_index;                    // 设备索引（在paired_inv_info中的位置）
    ota_state_t state;                       // 当前OTA升级状态
    uint16_t current_packet;                 // 当前数据包序号
    uint16_t total_packets;                  // 总数据包数
    uint8_t retry_count;                     // 当前命令重试次数（单次命令最多3次）
    uint8_t device_retry_count;              // 设备级重试次数（设备失败后重试最多5次）
    uint32_t timeout_counter;                // 超时计数器（ms）
    uint32_t version_query_interval_counter; // 版本询问间隔计数器（ms）
    bool ota_send_success;                   // 单次OTA传输是否完成
} ota_device_info_t;

// ================= OTA管理器结构体 =================
typedef struct
{
    // 基本状态
    bool ota_in_progress;        // OTA进行中标志
    ota_fw_type_t fw_type;       // 固件类型
    sub1g_ota_type_t sub1g_type; // Sub1G协议类型

    // 固件信息
    uint32_t fw_length;                   // 固件总长度
    uint32_t fw_crc;                      // 固件CRC校验
    char fw_version[OTA_VERSION_MAX_LEN]; // 固件版本
    uint32_t fw_current_address;          // 当前传输地址
    uint16_t total_packets_sub1g;         // Sub1G协议总包数（按64字节分包）

    // 设备管理
    ota_device_info_t devices[INV_DEVICE_MAX_NUM]; // 设备OTA信息数组
    uint8_t need_ota_device_count;                 // 需要升级的设备数量
    uint8_t current_device_index;                  // 当前正在升级的设备索引
    uint8_t success_count;                         // 成功升级的设备数量
    uint8_t failed_count;                          // 失败的设备数量
    uint8_t original_device_count;                 // 升级开始时的设备数量

    // 失败设备重试管理
    uint8_t failed_device_list[INV_DEVICE_MAX_NUM]; // 失败设备索引列表
    uint8_t failed_device_count;                    // 失败设备数量
    uint8_t retry_round;                            // 当前重试轮次(每轮重试所有失败设备)
    uint8_t firmware_retry_count;                   // 固件级别重试计数器(部分成功时累计)

    // 缓冲区
    uint8_t fw_buffer[OTA_FIRMWARE_BUFFER_SIZE]; // WiFi接收的固件缓冲区
    uint16_t fw_buffer_valid_len;                // 缓冲区有效数据长度
    bool waiting_wifi_data;                      // 等待WiFi数据标志

    // 状态标志
    bool disable_broadcast;       // 禁止功率广播标志
    bool disable_property_report; // 禁止属性上报标志
    uint32_t last_activity_ms;    // 最后活动时间戳
} ota_manager_t;

// ================= 全局变量声明 =================
extern ota_manager_t g_ota_manager;

// ================= 函数声明 =================
void ota_manager_init(void);
void ota_manager_task(void);
ota_result_t ota_get_finish_status(void);
sub1g_ota_type_t ota_fw_type_to_sub1g_type(ota_fw_type_t fw_type);

// ================= OTA与wifi之间的互动函数 =================
ota_start_result_t ota_check_start_status(void);
void ota_copy_wifi_fw_data(const uint8_t *data, uint16_t length);

// ================= OTA与sub1g之间的互动发送函数 =================
void ota_send_sub1g_init_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type, uint32_t length);
void ota_send_sub1g_data_packet(uint32_t sub1g_addr, sub1g_ota_type_t type, uint16_t total_packets, uint16_t packet_num, const uint8_t *data, uint8_t data_len);
void ota_send_sub1g_cancel_cmd(uint32_t sub1g_addr, sub1g_ota_type_t type);
void ota_send_sub1g_query_version_cmd(uint32_t sub1g_addr);

// ================= OTA与sub1g之间的互动接收回复函数 =================
void ota_handle_sub1g_init_ack(uint32_t sub1g_addr, uint8_t status);
void ota_handle_sub1g_data_ack(uint32_t sub1g_addr, uint16_t packet_num, uint8_t status);
void ota_handle_sub1g_cancel(uint32_t sub1g_addr);
void ota_handle_sub1g_version_report(uint32_t sub1g_addr, const char *sub1g_version, const char *mcu_version);
void ota_handle_ct_sub1g_version_report(const char *ct_sub1g_version);
void ota_force_cancel(void);

// ================= 工具函数 =================
uint16_t ota_calculate_checksum(const uint8_t *data, uint16_t length);

#endif /* __OTA_MANAGER_H__ */
