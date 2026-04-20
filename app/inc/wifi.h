/*
 * wifi.h
 *
 *  Created on: 2025年7月2日
 *      Author: yao
 */

#ifndef APP_INC_WIFI_H_
#define APP_INC_WIFI_H_

#include "hc32_ll.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>


typedef enum
{
    WIFI_VALUE_TYPE_FALSE = 0,
    WIFI_VALUE_TYPE_TRUE,
    WIFI_VALUE_TYPE_INT,
    WIFI_VALUE_TYPE_FLOAT,
    WIFI_VALUE_TYPE_STRING,
    WIFI_VALUE_TYPE_NOT_FOUND,
} wifi_value_type_t;

typedef enum
{
    PROP_RUN_STATE = 0x00000001,
    PROP_A_PHASE_POWER,
    PROP_B_PHASE_POWER,
    PROP_C_PHASE_POWER,
} wifi_prop_t;

typedef enum
{
    UNPROV = 0,   // 未配置
    UAP = 1,      // 配置阶段
    OFFLINE = 2,  // 未连接到网络
    LOCAL = 3,    // 已连接到本地网络,未连接到云服务器
    CLOUD = 4,    // 连接到云服务器
    UPDATING = 5, // 固件更新
} wifi_status_t;

typedef struct
{
    char sn[16]; // CT板SN (15+1字节)
    char version[12];
    char mac[12];
    char ssid[24];
    char passwd[24];
    uint16_t rssi;
    wifi_status_t net;
    uint16_t ble_on_enable;
    uint16_t debug_enable;
} wifi_info_t;

typedef enum
{
    WIFI_TX_NULL = 0,       // 空闲状态，没有通信任务
    PROP_EVENT = 1,         // 属性事件上报命令
    GET_MAC = 2,            // 获取MAC地址命令
    GET_NET = 3,            // 获取网络状态命令
    GET_DOWN = 4,           // 获取下行消息命令
    GET_TIME = 5,           // 获取时间命令
    SET_DEVICE = 6,         // 设置设备信息命令
    SET_MCU_VERSION = 7,    // 设置MCU版本命令
    GET_VERSION = 8,        // 获取版本信息命令
    GET_RSSI = 9,           // 获取信号强度命令
    OTA_READY = 10,         // OTA升级准备命令
    SET_BLE_ONOFF = 11,     // 设置蓝牙开关命令
    GET_SLAVE_VERSION = 12, // 设置Slave设备版本命令
} wifi_tx_type_t;           // 枚举定义了WiFi模块的通信命令类型

typedef struct
{
    int siid;
    int piid;
    uint8_t flag;
    wifi_value_type_t value_type;
    long int value;
    float f_value;
    char *s_value;
} wifi_msg_params_t;

typedef struct
{
    uint16_t restore;
    uint16_t device;
    uint16_t mcu_version;
    uint16_t slave_version; // slave设备版本上报
    uint16_t version;
    uint16_t mac;
    uint16_t connect;
    uint16_t ble_onoff;
    uint32_t get_down_count;
    uint32_t prop_ct_count;          // CT设备属性上报计数器
    uint32_t prop_inv_count;         // 微逆属性上报计数器
    uint8_t immediate_report_pair;   // 立即上报配对列表标志
    uint8_t immediate_report_bind;   // 立即上报绑定SN标志
    uint8_t immediate_report_unbind; // 立即上报解绑标志
    uint32_t get_net_count;
    uint32_t get_rssi_count;
    uint32_t get_time_count;
    uint32_t offline_count;
    uint32_t timeout_count;
    uint32_t prop_properties;
    uint32_t ble_off_count;
    uint32_t wait_result_timeout;
    uint32_t comm_timeout_count;      // WiFi通信超时计数器
    uint32_t version_check_count;     // 版本检查计数器
    uint32_t slave_version_report_ms; // 每1小时上报一次 slave版本
} wifi_tx_flag_t;

void wifi_init(void);
void wifi_task(void);
void wifi_timer_1ms_excute(void);
void check_wifi_communication(void);
void update_slave_versions(void);

extern wifi_tx_flag_t tx_flag;
extern wifi_tx_type_t tx_type;
extern wifi_info_t wifi_info;

extern void update_slave_versions(void);

#endif /* APP_INC_WIFI_H_ */
