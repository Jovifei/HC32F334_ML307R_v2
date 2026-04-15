#include "iot.h"
#include "ml307r.h"
#include "main.h"
#include "eeprom.h"
#include "sub1g.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

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

// 辅助函数：从payload中提取指定siid/piid的属性值
// 返回值格式: {"siid":x,"piid":y,"value":z,"code":0} 或 {"siid":x,"piid":y,"value":"str","code":0}
static int iot_get_single_property_value(char *out, int out_len, int siid, int piid) {
    float f_val = 0;
    int i_val = 0;
    bool is_string = false;
    char str_val[64] = {0};

    // INV设备 (siid 4-11)
    if (siid >= SIID_MIN && siid <= SIID_MAX) {
        uint8_t inv_idx = siid - SIID_MIN;
        if (!sys_param.paired_inv_info[inv_idx].is_valid) {
            // 设备无效，返回code -4007 (device not found)
            return -4007;
        }

        switch (piid) {
        case INV_PIID_ONLINE_STATE:
            i_val = sys_param.paired_inv_info[inv_idx].online_state;
            break;
        case INV_PIID_DEVICE_SN:
            snprintf(str_val, sizeof(str_val), "%s", sys_param.paired_inv_info[inv_idx].device_sn);
            is_string = true;
            break;
        case INV_PIID_SW_VERSION:
            snprintf(str_val, sizeof(str_val), "%s", sys_param.paired_inv_info[inv_idx].sw_version);
            is_string = true;
            break;
        case INV_PIID_SUB1G_VERSION:
            snprintf(str_val, sizeof(str_val), "%s", sys_param.paired_inv_info[inv_idx].sub1g_version);
            is_string = true;
            break;
        case INV_PIID_PRODUCT_MODEL:
            snprintf(str_val, sizeof(str_val), "%s", sys_param.paired_inv_info[inv_idx].product_model);
            is_string = true;
            break;
        case INV_PIID_WORK_STATE:
            i_val = sys_param.paired_inv_info[inv_idx].work_state;
            break;
        case INV_PIID_GRID_POWER:
            f_val = sys_param.paired_inv_info[inv_idx].grid_power;
            break;
        case INV_PIID_TODAY_ENERGY:
            f_val = sys_param.paired_inv_info[inv_idx].today_energy;
            break;
        case INV_PIID_LIFETIME_ENERGY:
            f_val = sys_param.paired_inv_info[inv_idx].lifetime_energy;
            break;
        case INV_PIID_ANTIFLOW_ENABLE:
            i_val = sys_param.paired_inv_info[inv_idx].antiflow_enable;
            break;
        case INV_PIID_INV_POWER_ENABLE:
            i_val = sys_param.paired_inv_info[inv_idx].power_enable;
            break;
        case INV_PIID_TODAY_POWER_TIME:
            f_val = sys_param.paired_inv_info[inv_idx].today_power_time;
            break;
        case INV_PIID_FAULT_PARAM:
            i_val = sys_param.paired_inv_info[inv_idx].fault_param;
            break;
        case INV_PIID_TEMPERATURE:
            f_val = sys_param.paired_inv_info[inv_idx].ambient_temperature;
            break;
        case INV_PIID_PV1_VOLTAGE:
            f_val = sys_param.paired_inv_info[inv_idx].pv[0].voltage;
            break;
        case INV_PIID_PV1_CURRENT:
            f_val = sys_param.paired_inv_info[inv_idx].pv[0].current;
            break;
        case INV_PIID_PV2_VOLTAGE:
            f_val = sys_param.paired_inv_info[inv_idx].pv[1].voltage;
            break;
        case INV_PIID_PV2_CURRENT:
            f_val = sys_param.paired_inv_info[inv_idx].pv[1].current;
            break;
        case INV_PIID_GRID_VOLTAGE:
            f_val = sys_param.paired_inv_info[inv_idx].grid_voltage;
            break;
        case INV_PIID_GRID_FREQUENCY:
            f_val = sys_param.paired_inv_info[inv_idx].grid_frequency;
            break;
        case INV_PIID_POWER_PHASE:
            i_val = sys_param.paired_inv_info[inv_idx].phase;
            break;
        case INV_PIID_POWER_LIMIT:
            i_val = sys_param.paired_inv_info[inv_idx].power_limit;
            break;
        case INV_PIID_CONNECTION_POINT:
            i_val = sys_param.paired_inv_info[inv_idx].connection_point;
            break;
        case INV_PIID_PV1_POWER:
            i_val = sys_param.paired_inv_info[inv_idx].pv[0].power;
            break;
        case INV_PIID_PV2_POWER:
            i_val = sys_param.paired_inv_info[inv_idx].pv[1].power;
            break;
        case INV_PIID_PV3_POWER:
            i_val = sys_param.paired_inv_info[inv_idx].pv[2].power;
            break;
        case INV_PIID_PV4_POWER:
            i_val = sys_param.paired_inv_info[inv_idx].pv[3].power;
            break;
        case INV_PIID_FFT_ENABLE:
            return -4003; // property not found (no equivalent field in current inv_device_t)
        case INV_PIID_PV_NUM:
            i_val = sys_param.paired_inv_info[inv_idx].pv_num;
            break;
        case INV_PIID_SUBG_ADDR:
            i_val = sys_param.paired_inv_info[inv_idx].sub1g_addr;
            break;
        case INV_PIID_CHANNEL_INDEX:
            i_val = sys_param.paired_inv_info[inv_idx].channel_index;
            break;
        case INV_PIID_PACKET_LOSS_RATE:
            i_val = sys_param.paired_inv_info[inv_idx].plr;
            break;
        default:
            return -4003; // property not found
        }
    }
    // CT设备 SIID=1 (设备信息)
    else if (siid == 1) {
        (void)piid;
        return -4003; // property not found (no device_info in current sys_param_t)
    }
    // CT设备 SIID=2 (设备功率)
    else if (siid == 2) {
        switch (piid) {
        case 1:
            i_val = sys_param.state;
            break;
        case 2:
            f_val = sys_param.ct1.power.fix_dir_power;
            break;
        case 3:
            f_val = sys_param.ct2.power.fix_dir_power;
            break;
        case 4:
            f_val = sys_param.ct3.power.fix_dir_power;
            break;
        case 5:
            f_val = sys_param.ct_today_energy;
            break;
        case 6:
            f_val = (sys_param.ct1.power.fix_dir_power +
                     sys_param.ct2.power.fix_dir_power +
                     sys_param.ct3.power.fix_dir_power);
            break;
        case 7:
            f_val = sys_param.ct_today_power_time;
            break;
        case 8:
            return -4003; // property not found (no lifetime energy accumulator in current sys_param_t)
        case 9:
            i_val = sys_param.is_three_phase;
            break;
        case 10:
            return -4003; // property not found
        case 11:
            f_val = sys_param.ct_today_power_time;
            break;
        case 12:
            f_val = sys_param.ct_today_energy;
            break;
        case 13:
            i_val = sys_param.grid.phase_id.sequence_k;
            break;
        case 14:
            return -4003; // property not found
        case 15:
            return -4003; // property not found
        case 16:
            f_val = sys_param.grid.ua_vol_rms;
            break;
        case 17:
            f_val = sys_param.grid.grid_frequency;
            break;
        default:
            return -4003;
        }
    }
    // CT设备 SIID=3 (设备配置)
    else if (siid == 3) {
        switch (piid) {
        case 2:
            i_val = sys_param.fft_identify.is_ffting;
            break;
        case 3:
            snprintf(str_val, sizeof(str_val), "%s", get_paired_device_list_string());
            is_string = true;
            break;
        case 5:
            return -4003; // property not found (no bind_info in current sys_param_t)
        case 6:
            return -4003; // property not found
        case 8:
            return -4003; // property not found (no phase_type in current sys_param_t)
        case 9:
            i_val = sys_param.fft_identify.is_ffting;
            break;
        case 10:
            i_val = sys_param.power_work_mode;
            break;
        case 11:
            i_val = sys_param.to_grid_power_limit;
            break;
        case 12:
            i_val = sys_param.fft_identify.power;
            break;
        case 14:
            return -4003; // property not found (no ct_dir in current sys_param_t)
        case 15:
            i_val = sys_param.sub1g.channel_index;
            break;
        default:
            return -4003;
        }
    }
    else {
        return -4003;
    }

    // 构造输出
    if (is_string) {
        snprintf(out, out_len, "{\"siid\":%d,\"piid\":%d,\"value\":\"%s\",\"code\":0}", siid, piid, str_val);
    } else {
        // 检查是否为浮点数(有小数部分)
        if (f_val != 0 && (f_val < 1 || f_val != (int)f_val)) {
            snprintf(out, out_len, "{\"siid\":%d,\"piid\":%d,\"value\":%.2f,\"code\":0}", siid, piid, f_val);
        } else {
            snprintf(out, out_len, "{\"siid\":%d,\"piid\":%d,\"value\":%d,\"code\":0}", siid, piid, i_val);
        }
    }
    return 0;
}

static void iot_handle_get_properties(int msg_id, const char *payload) {
    char result_params[512] = {0};
    int param_count = 0;

    // 解析params数组: [{"siid":1,"piid":2},{"siid":1,"piid":3}]
    const char *p = payload;
    while ((p = strstr(p, "\"siid\"")) != NULL && param_count < 16) {
        int siid = 0, piid = 0;
        const char *start;

        // 提取siid值
        p = strchr(p, ':');
        if (p) {
            p++;
            start = p;
            while (*p >= '0' && *p <= '9') p++;
            char buf[8] = {0};
            int len = p - start < 7 ? p - start : 7;
            strncpy(buf, start, len);
            siid = atoi(buf);
        }

        // 提取piid值
        p = strstr(p, "\"piid\"");
        if (p) {
            p = strchr(p, ':');
            if (p) {
                p++;
                start = p;
                while (*p >= '0' && *p <= '9') p++;
                char buf[8] = {0};
                int len = p - start < 7 ? p - start : 7;
                strncpy(buf, start, len);
                piid = atoi(buf);
            }
        }

        // 获取属性值
        char param[128] = {0};
        int code = iot_get_single_property_value(param, sizeof(param), siid, piid);
        if (code != 0) {
            // 属性获取失败
            if (param_count > 0) strcat(result_params, ",");
            snprintf(param, sizeof(param), "{\"siid\":%d,\"piid\":%d,\"code\":%d}", siid, piid, code);
        }

        if (param_count > 0) strcat(result_params, ",");
        strcat(result_params, param);
        param_count++;

        // 继续查找下一个siid
        if (param_count < 16) {
            p = strstr(p, "},");
            if (p) p += 2;
        }
    }

    // 构造最终JSON
    snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
        "{\"id\":%d,\"method\":\"result\",\"params\":[%s]}", msg_id, result_params);

    ml307r_mqtt_publish("up", s_iot_json_buf, 1);
}

// 辅助函数：设置指定siid/piid的属性值
// 返回错误码: 0成功, 负值为错误码
static int iot_set_single_property_value(int siid, int piid, const char *value_str) {
    // INV设备 (siid 4-11) - 大部分只读，少数可写
    if (siid >= SIID_MIN && siid <= SIID_MAX) {
        uint8_t inv_idx = siid - SIID_MIN;
        if (!sys_param.paired_inv_info[inv_idx].is_valid) {
            return -4007; // device not found
        }

        switch (piid) {
        case INV_PIID_ANTIFLOW_ENABLE:
            sys_param.paired_inv_info[inv_idx].antiflow_enable = (value_str[0] == 't' || value_str[0] == '1');
            break;
        case INV_PIID_INV_POWER_ENABLE:
            sys_param.paired_inv_info[inv_idx].power_enable = (value_str[0] == 't' || value_str[0] == '1');
            break;
        case INV_PIID_POWER_LIMIT:
            sys_param.paired_inv_info[inv_idx].power_limit = atoi(value_str);
            break;
        case INV_PIID_FFT_ENABLE:
            return -4003; // property not found (no equivalent field)
        // 其他INV属性为只读
        default:
            return -4002; // read-only property
        }
    }
    // CT设备 SIID=1 (设备信息) - SN可写
    else if (siid == 1) {
        switch (piid) {
        case 2: // SW version - 只读
            return -4002;
        default:
            return -4003; // property not found
        }
    }
    // CT设备 SIID=2 (设备功率) - 大部分只读
    else if (siid == 2) {
        switch (piid) {
        // 只读属性
        default:
            return -4002; // read-only
        }
    }
    // CT设备 SIID=3 (设备配置) - 可写
    else if (siid == 3) {
        switch (piid) {
        case 1: // restore_sys
            sys_param.restore_sys = (value_str[0] == 't' || value_str[0] == '1');
            break;
        case 5: // unbind_device_by_sn - 特殊处理
            // 需要调用解绑逻辑，这里简化处理
            break;
        case 6: // restore_wifi_cmd
            sys_param.wifi.restore_wifi_cmd = atoi(value_str);
            break;
        case 10: // power_work_mode
            sys_param.power_work_mode = atoi(value_str);
            eeprom_save_set_param();
            break;
        case 11: // to_grid_power_limit
            sys_param.to_grid_power_limit = atoi(value_str);
            eeprom_save_set_param();
            break;
        case 12: // fft_identify_power
            sys_param.fft_identify.power = atoi(value_str);
            break;
        case 14: // reset_ct_dir
        {
            uint8_t dir = atoi(value_str);
            if (dir == 1) {
                sys_param.ct1.power.power_direction *= (-1);
                sys_param.ct2.power.power_direction *= (-1);
                sys_param.ct3.power.power_direction *= (-1);
            } else if (dir == 2) {
                sys_param.ct1.power.power_direction = 1;
            } else if (dir == 3) {
                sys_param.ct1.power.power_direction = -1;
            }
            break;
        }
        case 15: // sub1g channel
            sys_param.sub1g.channel_index = atoi(value_str);
            sub1g_send_force_channel();
            break;
        default:
            return -4003;
        }
    }
    else {
        return -4003;
    }

    return 0;
}

static void iot_handle_set_properties(int msg_id, const char *payload) {
    char result_params[512] = {0};
    int param_count = 0;

    // 解析params数组: [{"siid":1,"piid":1,"value":10},...]
    const char *p = payload;
    while ((p = strstr(p, "\"siid\"")) != NULL && param_count < 16) {
        int siid = 0, piid = 0;
        const char *start;
        char value_str[64] = {0};

        // 提取siid值
        p = strchr(p, ':');
        if (p) {
            p++;
            start = p;
            while (*p >= '0' && *p <= '9') p++;
            char buf[8] = {0};
            int len = p - start < 7 ? p - start : 7;
            strncpy(buf, start, len);
            siid = atoi(buf);
        }

        // 提取piid值
        p = strstr(p, "\"piid\"");
        if (p) {
            p = strchr(p, ':');
            if (p) {
                p++;
                start = p;
                while (*p >= '0' && *p <= '9') p++;
                char buf[8] = {0};
                int len = p - start < 7 ? p - start : 7;
                strncpy(buf, start, len);
                piid = atoi(buf);
            }
        }

        // 提取value值 - 可能是数字或字符串
        p = strstr(p, "\"value\"");
        if (p) {
            p = strchr(p, ':');
            if (p) {
                p++;
                // 跳过空格
                while (*p == ' ') p++;
                if (*p == '"') {
                    // 字符串值
                    p++;
                    start = p;
                    p = strchr(p, '"');
                    if (p) {
                        int len = p - start < 63 ? p - start : 63;
                        strncpy(value_str, start, len);
                    }
                } else {
                    // 数字值
                    start = p;
                    while (*p >= '0' && *p <= '9') p++;
                    int len = p - start < 63 ? p - start : 63;
                    strncpy(value_str, start, len);
                }
            }
        }

        // 设置属性值
        int code = iot_set_single_property_value(siid, piid, value_str);

        // 构造结果param
        char param[64];
        snprintf(param, sizeof(param), "{\"siid\":%d,\"piid\":%d,\"code\":%d}", siid, piid, code);
        if (param_count > 0) strcat(result_params, ",");
        strcat(result_params, param);
        param_count++;

        // 继续查找下一个siid
        if (param_count < 16) {
            p = strstr(p, "},");
            if (p) p += 2;
        }
    }

    // 构造最终JSON
    snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
        "{\"id\":%d,\"method\":\"result\",\"params\":[%s]}", msg_id, result_params);

    ml307r_mqtt_publish("up", s_iot_json_buf, 1);
}

// IoT action处理
// action调用格式: {"method":"action","params":{"siid":x,"aiid":y,"args":{...}},"id":123}
// 响应格式: {"id":123,"method":"result","params":{"code":0}}
static void iot_handle_action(int msg_id, const char *payload) {
    // 简化实现：解析siid和aiid，返回成功
    // 实际action处理可能涉及复杂操作，暂时返回成功

    // 查找siid
    int siid = 0, aiid = 0;
    const char *p = strstr(payload, "\"siid\"");
    if (p) {
        p = strchr(p, ':');
        if (p) siid = atoi(p + 1);
    }

    // 查找aiid (action id)
    p = strstr(payload, "\"aiid\"");
    if (p) {
        p = strchr(p, ':');
        if (p) aiid = atoi(p + 1);
    }

    // 构造响应 - 简化处理，都返回成功
    snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
        "{\"id\":%d,\"method\":\"result\",\"params\":{\"siid\":%d,\"aiid\":%d,\"code\":0}}",
        msg_id, siid, aiid);

    ml307r_mqtt_publish("up", s_iot_json_buf, 1);
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

// 辅助函数：向JSON缓冲区添加属性
static int iot_add_property_to_json(char *buf, int buf_len, int *pos, int siid, int piid, int value) {
    int written = snprintf(buf + *pos, buf_len - *pos,
        "%s{\"siid\":%d,\"piid\":%d,\"value\":%d}",
        (*pos > 0) ? "," : "", siid, piid, value);
    *pos += written;
    return written;
}

static int iot_add_float_property_to_json(char *buf, int buf_len, int *pos, int siid, int piid, float value) {
    int written = snprintf(buf + *pos, buf_len - *pos,
        "%s{\"siid\":%d,\"piid\":%d,\"value\":%.2f}",
        (*pos > 0) ? "," : "", siid, piid, value);
    *pos += written;
    return written;
}

static int iot_add_string_property_to_json(char *buf, int buf_len, int *pos, int siid, int piid, const char *value) {
    int written = snprintf(buf + *pos, buf_len - *pos,
        "%s{\"siid\":%d,\"piid\":%d,\"value\":\"%s\"}",
        (*pos > 0) ? "," : "", siid, piid, value);
    *pos += written;
    return written;
}

// 构造CT设备siid=2的属性JSON (功率数据)
static int iot_build_ct_siid2_json(char *buf, int buf_len, int *pos) {
    // piid1: 系统状态
    iot_add_property_to_json(buf, buf_len, pos, 2, 1, sys_param.state);

    // piid2: CT1功率
    float ct1_power = sys_param.ct1.power.fix_dir_power;
    if (!sys_param.is_three_phase && sys_param.ct1.status.connect_status != CT_STATUS_ONLINE) {
        ct1_power = 0;
    }
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 2, ct1_power);

    // piid3: CT2功率
    float ct2_power = sys_param.ct2.power.fix_dir_power;
    if (sys_param.ct2.status.connect_status != CT_STATUS_ONLINE) {
        ct2_power = 0;
    }
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 3, ct2_power);

    // piid4: CT3功率
    float ct3_power = sys_param.ct3.power.fix_dir_power;
    if (sys_param.ct3.status.connect_status != CT_STATUS_ONLINE) {
        ct3_power = 0;
    }
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 4, ct3_power);

    // piid5: 今日发电量
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 5, sys_param.ct_today_energy);

    // piid6: 电网总功率
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 6,
        (sys_param.ct1.power.fix_dir_power +
         sys_param.ct2.power.fix_dir_power +
         sys_param.ct3.power.fix_dir_power));

    // piid7: 今日发电时长
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 7, sys_param.ct_today_power_time);

    // piid9: 三相标识
    iot_add_property_to_json(buf, buf_len, pos, 2, 9, sys_param.is_three_phase);
    // piid16: 电网电压
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 16, sys_param.grid.ua_vol_rms);

    // piid17: 电网频率
    iot_add_float_property_to_json(buf, buf_len, pos, 2, 17, sys_param.grid.grid_frequency);

    return 0;
}

// 构造CT设备siid=3的属性JSON (配置数据)
static int iot_build_ct_siid3_json(char *buf, int buf_len, int *pos) {
    // piid3: 配对列表
    iot_add_string_property_to_json(buf, buf_len, pos, 3, 3, get_paired_device_list_string());

    // piid10: 功率模式
    iot_add_property_to_json(buf, buf_len, pos, 3, 10, sys_param.power_work_mode);

    // piid11: 馈网限制值
    iot_add_property_to_json(buf, buf_len, pos, 3, 11, sys_param.to_grid_power_limit);

    // piid15: Sub1G信道
    iot_add_property_to_json(buf, buf_len, pos, 3, 15, sys_param.sub1g.channel_index);

    return 0;
}

// 构造INV设备的属性JSON
static int iot_build_inv_properties_json(char *buf, int buf_len, int *pos, uint8_t inv_idx) {
    if (inv_idx >= INV_DEVICE_MAX_NUM || !sys_param.paired_inv_info[inv_idx].is_valid) {
        return -1;
    }

    uint8_t siid = sys_param.paired_inv_info[inv_idx].siid;
    if (siid < SIID_MIN || siid > SIID_MAX) {
        return -1;
    }

    // 在线状态
    iot_add_property_to_json(buf, buf_len, pos, siid, INV_PIID_ONLINE_STATE,
        sys_param.paired_inv_info[inv_idx].online_state);

    // 工作状态
    iot_add_property_to_json(buf, buf_len, pos, siid, INV_PIID_WORK_STATE,
        sys_param.paired_inv_info[inv_idx].work_state);

    // 电网功率
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_GRID_POWER,
        sys_param.paired_inv_info[inv_idx].grid_power);

    // 今日发电量
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_TODAY_ENERGY,
        sys_param.paired_inv_info[inv_idx].today_energy);

    // 累计发电量
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_LIFETIME_ENERGY,
        sys_param.paired_inv_info[inv_idx].lifetime_energy);

    // 防逆流使能
    iot_add_property_to_json(buf, buf_len, pos, siid, INV_PIID_ANTIFLOW_ENABLE,
        sys_param.paired_inv_info[inv_idx].antiflow_enable);

    // 逆变器使能
    iot_add_property_to_json(buf, buf_len, pos, siid, INV_PIID_INV_POWER_ENABLE,
        sys_param.paired_inv_info[inv_idx].power_enable);

    // 温度
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_TEMPERATURE,
        sys_param.paired_inv_info[inv_idx].ambient_temperature);

    // PV1电压
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_PV1_VOLTAGE,
        sys_param.paired_inv_info[inv_idx].pv[0].voltage);

    // PV1电流
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_PV1_CURRENT,
        sys_param.paired_inv_info[inv_idx].pv[0].current);

    // 电网电压
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_GRID_VOLTAGE,
        sys_param.paired_inv_info[inv_idx].grid_voltage);

    // 电网频率
    iot_add_float_property_to_json(buf, buf_len, pos, siid, INV_PIID_GRID_FREQUENCY,
        sys_param.paired_inv_info[inv_idx].grid_frequency);

    return 0;
}

// 构造properties_changed JSON
// 格式: {"id":123,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0},...]}
static char* iot_build_properties_changed_json(void) {
    static char json[1024];
    char params[768] = {0};
    int pos = 0;

    // 填充CT siid2属性
    iot_build_ct_siid2_json(params, sizeof(params), &pos);

    // 构造最终JSON
    snprintf(json, sizeof(json),
        "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[%s]}",
        (int)sys_param.date_broadcast_counter, params);
    return json;
}

// 构造指定INV设备的properties_changed JSON
static char* iot_build_inv_properties_changed_json(uint8_t inv_idx) {
    static char json[1024];
    char params[512] = {0};
    int pos = 0;

    if (iot_build_inv_properties_json(params, sizeof(params), &pos, inv_idx) != 0) {
        // 无效设备，返回空params
        snprintf(json, sizeof(json),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[]}",
            (int)sys_param.date_broadcast_counter);
    } else {
        snprintf(json, sizeof(json),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[%s]}",
            (int)sys_param.date_broadcast_counter, params);
    }
    return json;
}

static void iot_publish_properties_changed(const char *json) {
    if (json) {
        ml307r_mqtt_publish("up", json, 1);
    }
}

void iot_report_immediate(uint8_t inv_idx) {
    char *json = iot_build_inv_properties_changed_json(inv_idx);
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
    return iot_build_inv_properties_changed_json(inv_idx);
}

void iot_trigger_ct_report(void) {
    static uint8_t s_ct_siid_toggle = 0;
    uint8_t siid = (s_ct_siid_toggle == 0) ? 2 : 3;
    s_ct_siid_toggle = (s_ct_siid_toggle + 1) % 2;

    char *json;
    if (siid == 2) {
        json = iot_build_properties_changed_json();
    } else {
        // Build siid=3 properties
        static char siid3_json[1024];
        char params[512] = {0};
        int pos = 0;
        iot_build_ct_siid3_json(params, sizeof(params), &pos);
        snprintf(siid3_json, sizeof(siid3_json),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[%s]}",
            (int)sys_param.date_broadcast_counter, params);
        json = siid3_json;
    }

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
    if (!s_iot_connected) {
        // 连接建立时打印一次
        s_iot_connected = true;
        DEBUG_PRINTF("[IOT] connected\r\n");
    }

    // 1. 处理即时上报 - 绑定
    if (iot_tx_flag.immediate_report_bind) {
        iot_tx_flag.immediate_report_bind = 0;
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":5,\"value\":\"%s\"}]}",
            (int)sys_param.date_broadcast_counter, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }

    // 2. 处理即时上报 - 解绑
    if (iot_tx_flag.immediate_report_unbind) {
        iot_tx_flag.immediate_report_unbind = 0;
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":6,\"value\":\"%s\"}]}",
            (int)sys_param.date_broadcast_counter, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }

    // 3. CT设备定时上报(5分钟)
    iot_tx_flag.prop_ct_count += 100;
    if (iot_tx_flag.prop_ct_count >= PROP_CT_MS_INTERVAL) {
        iot_tx_flag.prop_ct_count = 0;
        iot_trigger_ct_report();
    }

    // 4. INV设备定时上报(3分钟)
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
