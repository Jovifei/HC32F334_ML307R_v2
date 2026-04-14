#include "iot.h"
#include "ml307r.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

// MQTT JSON消息缓冲区
static char s_iot_json_buf[512];

// IoT上报标志
iot_tx_flag_t iot_tx_flag = {
    .prop_ct_count = 0,
    .prop_inv_count = 0,
    .immediate_report_bind = 0,
    .immediate_report_unbind = 0,
};

// 配对/解绑SN缓存
static char s_report_bind_sn[16];

// MQTT连接状态
static bool s_iot_connected = false;

// 上报计数器阈值
#define PROP_CT_MS_INTERVAL    300000  // CT定时上报: 5分钟
#define PROP_INV_MS_INTERVAL   180000 // INV定时上报: 3分钟

void iot_mqtt_downlink_handler(const char *topic, const char *payload) {
    (void)topic;
    (void)payload;
}

void iot_report_immediate(uint8_t inv_idx) {
    (void)inv_idx;
}

void iot_report_bind(const char *sn) {
    (void)sn;
}

void iot_report_unbind(const char *sn) {
    (void)sn;
}

void iot_trigger_ct_report(void) {
}

void iot_trigger_inverter_report(uint8_t inv_idx) {
    (void)inv_idx;
}

bool iot_is_connected(void) {
    return s_iot_connected;
}

void iot_task(void) {
    // 空函数，后续任务完善
}
