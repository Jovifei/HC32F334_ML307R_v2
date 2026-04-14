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

// 简单的JSON解析：提取method和id
// payload格式: {"method":"get_properties","params":[...],"id":123}
// 返回0成功，-1失败
static int iot_json_parse_method(const char *payload, char *method, int method_len, int *msg_id) {
    const char *p = payload;
    const char *method_start;
    const char *id_start;
    char id_buf[16] = {0};

    if (payload == NULL || method == NULL || msg_id == NULL) {
        return -1;
    }

    // 查找 "method":"
    p = strstr(payload, "\"method\":\"");
    if (p == NULL) {
        return -1;
    }
    p += strlen("\"method\":\"");
    method_start = p;

    // 查找 method 值的结束引号
    p = strchr(p, '"');
    if (p == NULL) {
        return -1;
    }

    // 复制 method
    int method_copy_len = (p - method_start < method_len - 1) ? (p - method_start) : (method_len - 1);
    strncpy(method, method_start, method_copy_len);
    method[method_copy_len] = '\0';

    // 查找 "id":
    p = strstr(p, "\"id\":");
    if (p == NULL) {
        return -1;
    }
    p += strlen("\"id\":");

    // 跳过空格
    while (*p == ' ') {
        p++;
    }

    // 提取 id 值
    id_start = p;
    p = strchr(p, ',');
    if (p == NULL) {
        // id可能是最后一个字段，查找 }
        p = strchr(id_start, '}');
    }
    if (p == NULL) {
        return -1;
    }

    int id_len = (p - id_start < sizeof(id_buf) - 1) ? (p - id_start) : (sizeof(id_buf) - 1);
    strncpy(id_buf, id_start, id_len);
    id_buf[id_len] = '\0';

    *msg_id = atoi(id_buf);
    return 0;
}

static void iot_handle_get_properties(int msg_id, const char *payload) {
    (void)msg_id;
    (void)payload;
    // TODO: 完整实现
}

static void iot_handle_set_properties(int msg_id, const char *payload) {
    (void)msg_id;
    (void)payload;
    // TODO: 完整实现
}

static void iot_handle_action(int msg_id, const char *payload) {
    (void)msg_id;
    (void)payload;
    // TODO: 完整实现
}

void iot_mqtt_downlink_handler(const char *topic, const char *payload) {
    char method[32] = {0};
    int msg_id = 0;

    if (iot_json_parse_method(payload, method, sizeof(method), &msg_id) != 0) {
        return;
    }

    if (strcmp(method, "get_properties") == 0) {
        iot_handle_get_properties(msg_id, payload);
    } else if (strcmp(method, "set_properties") == 0) {
        iot_handle_set_properties(msg_id, payload);
    } else if (strcmp(method, "action") == 0) {
        iot_handle_action(msg_id, payload);
    }
}

// 构造properties_changed JSON
// 格式: {"id":123,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0},...]}
static char* iot_build_properties_changed_json(void) {
    // 返回一个示例JSON，实际属性填充后续完善
    static char json[512];
    snprintf(json, sizeof(json),
        "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[]}",
        (int)sys_param.current_time_ms);
    return json;
}

static void iot_publish_properties_changed(const char *json) {
    if (json) {
        ml307r_mqtt_publish("up", json, 1);
    }
}

void iot_report_immediate(uint8_t inv_idx) {
    (void)inv_idx;
    char *json = iot_build_properties_changed_json();
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

void iot_report_bind(const char *sn) {
    if (!sn) return;
    snprintf(s_report_bind_sn, sizeof(s_report_bind_sn), "%s", sn);
    iot_tx_flag.immediate_report_bind = 1;
}

void iot_report_unbind(const char *sn) {
    if (!sn) return;
    snprintf(s_report_bind_sn, sizeof(s_report_bind_sn), "%s", sn);
    iot_tx_flag.immediate_report_unbind = 1;
}

// 辅助函数 - 获取指定INV的JSON
static char* iot_build_properties_json_with_idx(uint8_t inv_idx) {
    (void)inv_idx;
    return iot_build_properties_changed_json();
}

void iot_trigger_ct_report(void) {
    char *json = iot_build_properties_changed_json();
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

void iot_trigger_inverter_report(uint8_t inv_idx) {
    (void)inv_idx;
    char *json = iot_build_properties_json_with_idx(inv_idx);
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

bool iot_is_connected(void) {
    return s_iot_connected;
}

void iot_task(void) {
    if (!ml307r_mqtt_is_connected()) {
        s_iot_connected = false;
        return;
    }
    s_iot_connected = true;

    // 1. 处理即时上报 - 绑定
    if (iot_tx_flag.immediate_report_bind) {
        iot_tx_flag.immediate_report_bind = 0;
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":5,\"value\":\"%s\"}]}",
            (int)sys_param.current_time_ms, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }

    // 2. 处理即时上报 - 解绑
    if (iot_tx_flag.immediate_report_unbind) {
        iot_tx_flag.immediate_report_unbind = 0;
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":6,\"value\":\"%s\"}]}",
            (int)sys_param.current_time_ms, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }

    // 3. 定时计数器递增 (假设100ms调用一次)
    static uint32_t s_timer_tick = 0;
    s_timer_tick += 100;

    // 4. CT设备定时上报(5分钟)
    if (s_timer_tick >= 100) {
        iot_tx_flag.prop_ct_count += 100;
        if (iot_tx_flag.prop_ct_count >= PROP_CT_MS_INTERVAL) {
            iot_tx_flag.prop_ct_count = 0;
            iot_trigger_ct_report();
        }
    }

    // 5. INV设备定时上报(3分钟)
    if (s_timer_tick >= 100) {
        iot_tx_flag.prop_inv_count += 100;
        if (iot_tx_flag.prop_inv_count >= PROP_INV_MS_INTERVAL) {
            iot_tx_flag.prop_inv_count = 0;
            for (uint8_t i = 0; i < INV_DEVICE_MAX_NUM; i++) {
                if (sys_param.paired_inv_info[i].is_valid) {
                    iot_trigger_inverter_report(i);
                }
            }
        }
    }
}
