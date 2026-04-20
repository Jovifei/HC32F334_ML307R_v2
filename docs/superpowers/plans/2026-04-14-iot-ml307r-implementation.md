# IoT ML307R Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace wifi.c/wifi.h with iot.c/iot.h, using ML307R MQTT interface instead of ESP32 UART, while preserving properties_changed/set_properties/get_properties functionality.

**Architecture:** Create iot.c/iot.h as a new IoT abstraction layer that communicates with cloud via ML307R MQTT. The module provides well-封装好- interfaces for: MQTT下行JSON解析, MQTT上行JSON构造, 属性上报调度. Reuse property handling logic from wifi.c with JSON format adaptation.

**Tech Stack:** C language, ML307R 4G module, MQTT protocol, cJSON library (for JSON parsing/construction)

---

## File Structure

| File | Responsibility |
|------|----------------|
| `app/inc/iot.h` | IoT module header - task interface, data structures, function declarations |
| `app/src/iot.c` | IoT module implementation - MQTT handling, property reporting |
| `app/src/ml307r.h` | Add mqtt_set_downlink_callback() interface |
| `app/src/ml307r.c` | Implement MQTT callback mechanism |
| `app/src/wifi.c` | To be deleted after iot.c is complete |
| `app/inc/wifi.h` | To be deleted after iot.h is complete |

---

## Task 1: Create iot.h - IoT Module Header

**Files:**
- Create: `app/inc/iot.h`

- [ ] **Step 1: Create iot.h with basic definitions**

```c
#ifndef IOT_H
#define IOT_H

#include <stdbool.h>
#include <stdint.h>

// IoT任务初始化（MQTT连接后由ml307r调用）
void iot_task(void);

// MQTT下行处理回调（供ml307r调用）
void iot_mqtt_downlink_handler(const char *topic, const char *payload);

// 即时属性上报接口（供其他模块调用）
void iot_report_immediate(uint8_t inv_idx);        // INV立即属性上报
void iot_report_bind(const char *sn);             // 配对绑定上报
void iot_report_unbind(const char *sn);           // 解绑上报

// 定时属性上报触发（供ml307r_task或main调用）
void iot_trigger_ct_report(void);                 // 触发CT定时上报
void iot_trigger_inverter_report(uint8_t inv_idx); // 触发INV定时上报

// 获取IoT模块状态
bool iot_is_connected(void);

// 上报标志结构体（复用wifi.c逻辑）
typedef struct {
    uint32_t prop_ct_count;          // CT定时上报计数器(ms)
    uint32_t prop_inv_count;        // INV定时上报计数器(ms)
    uint8_t immediate_report_bind;   // 绑定立即上报标志
    uint8_t immediate_report_unbind; // 解绑立即上报标志
} iot_tx_flag_t;

extern iot_tx_flag_t iot_tx_flag;

#endif // IOT_H
```

- [ ] **Step 2: Commit**

```bash
git add app/inc/iot.h
git commit -m "feat(iot): add iot.h header file with task interface"
```

---

## Task 2: Create iot.c - Core Module Implementation

**Files:**
- Create: `app/src/iot.c`

- [ ] **Step 1: Create iot.c with includes and global variables**

```c
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
```

- [ ] **Step 2: Commit**

```bash
git add app/src/iot.c
git commit -m "feat(iot): add iot.c with global variables and includes"
```

---

## Task 3: Implement iot_mqtt_downlink_handler - MQTT下行处理

**Files:**
- Modify: `app/src/iot.c`

- [ ] **Step 1: Add JSON parsing helper and downlink handler**

```c
// 简单的JSON解析辅助函数
// 解析 {"method":"get_properties","params":[...],"id":123}
// 返回值: 成功返回true, payload必须是只读
static int iot_json_parse_method(const char *payload, char *method, int method_len, int *msg_id) {
    const char *p = payload;
    
    // 查找 "method"
    const char *m = strstr(p, "\"method\"");
    if (!m) return -1;
    
    // 提取method值
    m = strchr(m, ':');
    if (!m) return -1;
    m++;
    while (*m == ' ' || *m == '\"') m++;
    
    int i = 0;
    while (*m && *m != '\"' && i < method_len - 1) {
        method[i++] = *m++;
    }
    method[i] = '\0';
    
    // 查找 "id"
    const char *id = strstr(p, "\"id\"");
    if (id) {
        id = strchr(id, ':');
        if (id) {
            id++;
            while (*id == ' ' || *id == '\"') id++;
            *msg_id = atoi(id);
        }
    }
    
    return 0;
}

// MQTT下行处理回调
void iot_mqtt_downlink_handler(const char *topic, const char *payload) {
    char method[32] = {0};
    int msg_id = 0;
    
    if (iot_json_parse_method(payload, method, sizeof(method), &msg_id) != 0) {
        return;
    }
    
    if (strcmp(method, "get_properties") == 0) {
        // 处理get_properties
        iot_handle_get_properties(msg_id, payload);
    } else if (strcmp(method, "set_properties") == 0) {
        // 处理set_properties
        iot_handle_set_properties(msg_id, payload);
    } else if (strcmp(method, "action") == 0) {
        // 处理action
        iot_handle_action(msg_id, payload);
    }
}
```

- [ ] **Step 2: Add stub handlers for get/set/action**

```c
// 处理get_properties
static void iot_handle_get_properties(int msg_id, const char *payload) {
    // 解析siid/piid数组，构造result JSON响应
    // 复用wifi.c的wifi_properties_para()和wifi_get_properties_reply()逻辑
    // 格式: {"id":123,"method":"result","params":[{"siid":1,"piid":2,"value":10,"code":0},...]}
    
    // TODO: 完整实现
    // 1. 解析params数组中的siid/piid
    // 2. 调用wifi_get_properties_reply()获取属性值
    // 3. 构造result JSON
    // 4. ml307r_mqtt_publish("up", json, 1);
}

// 处理set_properties  
static void iot_handle_set_properties(int msg_id, const char *payload) {
    // 解析siid/piid/value数组，设置sys_param，构造result JSON响应
    // 复用wifi.c的wifi_properties_para()和wifi_set_properties_reply()逻辑
    
    // TODO: 完整实现
}

// 处理action
static void iot_handle_action(int msg_id, const char *payload) {
    // 解析action调用，构造result JSON响应
    
    // TODO: 完整实现
}
```

- [ ] **Step 3: Commit**

```bash
git add app/src/iot.c
git commit -m "feat(iot): add iot_mqtt_downlink_handler with get/set/action stubs"
```

---

## Task 4: Implement iot_report functions - 即时上报接口

**Files:**
- Modify: `app/src/iot.c`

- [ ] **Step 1: Add properties_changed JSON构造**

```c
// 构造properties_changed JSON
// 格式: {"id":123,"method":"properties_changed","params":[{"siid":2,"piid":1,"value":0},...]}
static char* iot_build_properties_changed_json(void) {
    static char json[512];
    int len = 0;
    
    // TODO: 复用wifi.c的report_xxx_properties()函数逻辑
    // 但输出格式改为JSON而不是文本
    
    return json;
}

// 发送properties_changed到MQTT
static void iot_publish_properties_changed(const char *json) {
    ml307r_mqtt_publish("up", json, 1);
}
```

- [ ] **Step 2: Implement iot_report_immediate**

```c
// 即时属性上报 - INV状态/告警变化时调用
void iot_report_immediate(uint8_t inv_idx) {
    // 复用wifi.c的report_inverter_properties()函数逻辑
    // 构造JSON并发送
    
    char *json = iot_build_properties_changed_json();
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

// 配对绑定上报
void iot_report_bind(const char *sn) {
    if (!sn) return;
    
    snprintf(s_report_bind_sn, sizeof(s_report_bind_sn), "%s", sn);
    iot_tx_flag.immediate_report_bind = 1;
}

// 解绑上报
void iot_report_unbind(const char *sn) {
    if (!sn) return;
    
    snprintf(s_report_bind_sn, sizeof(s_report_bind_sn), "%s", sn);
    iot_tx_flag.immediate_report_unbind = 1;
}
```

- [ ] **Step 3: Commit**

```bash
git add app/src/iot.c
git commit -m "feat(iot): add iot_report_immediate/bind/unbind functions"
```

---

## Task 5: Implement iot_task - 主任务循环

**Files:**
- Modify: `app/src/iot.c`

- [ ] **Step 1: Add iot_task main loop**

```c
// IoT主任务（MQTT连接后由ml307r调用）
void iot_task(void) {
    if (!ml307r_mqtt_is_connected()) {
        s_iot_connected = false;
        return;
    }
    s_iot_connected = true;
    
    // 1. 处理即时上报
    if (iot_tx_flag.immediate_report_bind) {
        iot_tx_flag.immediate_report_bind = 0;
        // 构造 {"method":"properties_changed","params":[{"siid":3,"piid":5,"value":"SN"}]}
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":5,\"value\":\"%s\"}]}",
            (int)sys_param.current_time_ms, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }
    
    if (iot_tx_flag.immediate_report_unbind) {
        iot_tx_flag.immediate_report_unbind = 0;
        snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
            "{\"id\":%d,\"method\":\"properties_changed\",\"params\":[{\"siid\":3,\"piid\":6,\"value\":\"%s\"}]}",
            (int)sys_param.current_time_ms, s_report_bind_sn);
        iot_publish_properties_changed(s_iot_json_buf);
    }
    
    // 2. 处理定时上报 - CT设备(5分钟)
    iot_tx_flag.prop_ct_count += 100;  // 假设100ms调用一次
    if (iot_tx_flag.prop_ct_count >= PROP_CT_MS_INTERVAL) {
        iot_tx_flag.prop_ct_count = 0;
        iot_trigger_ct_report();
    }
    
    // 3. 处理定时上报 - INV设备(3分钟)
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
```

- [ ] **Step 2: Add iot_trigger functions**

```c
// 触发CT定时上报
void iot_trigger_ct_report(void) {
    // 复用wifi.c的report_ct_siid2/3_properties()函数逻辑
    // 交替上报siid2和siid3
    
    static uint8_t ct_toggle = 0;
    if (ct_toggle == 0) {
        // report_ct_siid3_properties -> JSON
        ct_toggle = 1;
    } else {
        // report_ct_siid2_properties -> JSON
        ct_toggle = 0;
    }
    
    char *json = iot_build_properties_changed_json();
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

// 触发INV定时上报
void iot_trigger_inverter_report(uint8_t inv_idx) {
    // 复用wifi.c的report_inverter_properties_scheduled()函数逻辑
    
    char *json = iot_build_properties_changed_json();
    if (json && strlen(json) > 10) {
        iot_publish_properties_changed(json);
    }
}

// 获取IoT连接状态
bool iot_is_connected(void) {
    return s_iot_connected;
}
```

- [ ] **Step 3: Commit**

```bash
git add app/src/iot.c
git commit -m "feat(iot): implement iot_task main loop and trigger functions"
```

---

## Task 6: Modify ml307r.h - Add MQTT Callback Interface

**Files:**
- Modify: `app/src/ml307r.h`

- [ ] **Step 1: Add callback interface declarations**

```c
// 在 ml307r.h 文件末尾添加以下内容（在 #endif 之前）

//============================================================================
// MQTT回调接口
//============================================================================

/**
 * MQTT下行数据回调类型
 * @param topic 主题名称 (如 "down")
 * @param payload JSON数据指针 (只读)
 */
typedef void (*mqtt_downlink_cb)(const char *topic, const char *payload);

/**
 * 设置MQTT下行回调
 * @param cb 回调函数指针，MQTT收到down主题消息时调用
 */
void ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb);

/**
 * 获取MQTT连接状态
 * @return true=已连接, false=未连接
 */
bool ml307r_mqtt_is_connected(void);
```

- [ ] **Step 2: Commit**

```bash
git add app/src/ml307r.h
git commit -m "feat(ml307r): add mqtt_set_downlink_callback and is_connected interfaces"
```

---

## Task 7: Modify ml307r.c - Implement Callback Mechanism

**Files:**
- Modify: `app/src/ml307r.c`

- [ ] **Step 1: Add static callback variable**

```c
// 在文件顶部添加（状态变量附近）
static mqtt_downlink_cb s_mqtt_downlink_cb = NULL;
```

- [ ] **Step 2: Implement ml307r_mqtt_set_downlink_callback**

```c
void ml307r_mqtt_set_downlink_callback(mqtt_downlink_cb cb) {
    s_mqtt_downlink_cb = cb;
}
```

- [ ] **Step 3: Implement ml307r_mqtt_is_connected**

```c
bool ml307r_mqtt_is_connected(void) {
    return s_mqtt_state == MQTT_STATE_CONNECTED;
}
```

- [ ] **Step 4: Add uplink handler call in MQTT receive**

```c
// 在MQTT收到消息的回调中，添加对s_mqtt_downlink_cb的调用
// 具体位置取决于MQTT收到消息的处理位置

void ml307r_mqtt_on_message(const char *topic, const char *payload) {
    // 如果是down主题，调用下行回调
    if (strstr(topic, "down") != NULL && s_mqtt_downlink_cb) {
        s_mqtt_downlink_cb(topic, payload);
    }
}
```

- [ ] **Step 5: Commit**

```bash
git add app/src/ml307r.c
git commit -m "feat(ml307r): implement mqtt callback mechanism"
```

---

## Task 8: Connect iot_task to ml307r - Integration

**Files:**
- Modify: `app/src/ml307r.c` (mqtt_connect success section)

- [ ] **Step 1: In ml307r_mqtt_connect success callback, set iot callback**

```c
// 在MQTT连接成功的位置，添加：
ml307r_mqtt_set_downlink_callback(iot_mqtt_downlink_handler);

// 并确保在主循环中调用iot_task：
// 在ml307r_task()中，当MQTT已连接时调用
if (ml307r_mqtt_is_connected()) {
    iot_task();  // 添加这一行
}
```

- [ ] **Step 2: Commit**

```bash
git add app/src/ml307r.c
git commit -m "feat(ml307r): connect iot_task to mqtt connected callback"
```

---

## Task 9: Final Integration - iot.c Full Property Logic

**Files:**
- Modify: `app/src/iot.c`

- [ ] **Step 1: Complete iot_handle_get_properties**

```c
static void iot_handle_get_properties(int msg_id, const char *payload) {
    // 解析params数组: [{"siid":1,"piid":2},{"siid":1,"piid":3}]
    // 构造result JSON: {"id":123,"method":"result","params":[{"siid":1,"piid":2,"value":10,"code":0}]}
    
    // 简化实现：遍历最多8个siid/piid对
    char result_params[256] = {0};
    int param_count = 0;
    
    // TODO: 完整的JSON解析和属性获取逻辑
    // 参考wifi.c的wifi_properties_para()和wifi_get_properties_reply()
    
    // 构造最终JSON
    snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
        "{\"id\":%d,\"method\":\"result\",\"params\":[%s]}",
        msg_id, result_params);
    
    ml307r_mqtt_publish("up", s_iot_json_buf, 1);
}
```

- [ ] **Step 2: Complete iot_handle_set_properties**

```c
static void iot_handle_set_properties(int msg_id, const char *payload) {
    // 解析params数组: [{"siid":1,"piid":1,"value":10},...]
    // 设置sys_param对应字段
    // 构造result JSON: {"id":123,"method":"result","params":[{"siid":1,"piid":1,"code":0}]}
    
    char result_params[256] = {0};
    
    // TODO: 完整的JSON解析、参数设置、阈值校验逻辑
    // 参考wifi.c的wifi_properties_para()和wifi_set_properties_reply()
    
    snprintf(s_iot_json_buf, sizeof(s_iot_json_buf),
        "{\"id\":%d,\"method\":\"result\",\"params\":[%s]}",
        msg_id, result_params);
    
    ml307r_mqtt_publish("up", s_iot_json_buf, 1);
}
```

- [ ] **Step 3: Complete iot_build_properties_changed_json**

```c
static char* iot_build_properties_changed_json(void) {
    // 复用wifi.c的report_xxx_properties()函数内容
    // 但将输出格式从文本改为JSON
    
    // 例如 report_inverter_properties() 原本输出:
    // "properties_changed 4 1 1 4 2 1 4 3 0 ..."  (文本格式)
    //
    // 现在应该输出:
    // {"id":123,"method":"properties_changed","params":[
    //   {"siid":4,"piid":1,"value":1},
    //   {"siid":4,"piid":2,"value":1},
    //   ...]}
    
    // TODO: 实现完整的JSON构造逻辑
    
    return s_iot_json_buf;
}
```

- [ ] **Step 4: Commit**

```bash
git add app/src/iot.c
git commit -m "feat(iot): implement full property get/set/build logic"
```

---

## Task 10: Cleanup - Delete Old wifi.c and wifi.h

**Files:**
- Delete: `app/src/wifi.c`, `app/inc/wifi.h`

- [ ] **Step 1: Delete wifi.c and wifi.h**

```bash
git rm app/src/wifi.c app/inc/wifi.h
```

- [ ] **Step 2: Commit**

```bash
git commit -m "refactor: remove old wifi.c/wifi.h, replaced by iot.c/iot.h"
```

---

## Task 11: Update main.c - Use iot instead of wifi

**Files:**
- Modify: `app/src/main.c`

- [ ] **Step 1: Replace wifi.h include with iot.h**

```c
// 将 #include "wifi.h" 改为 #include "iot.h"
```

- [ ] **Step 2: Ensure iot_task is called in main loop**

```c
// 在main函数或主循环中，确保调用ml307r_task()
```

- [ ] **Step 3: Commit**

```bash
git add app/src/main.c
git commit -m "feat(main): use iot instead of wifi"
```

---

## Verification Checklist

- [ ] iot.h created with all required interfaces
- [ ] iot.c implements iot_task(), iot_mqtt_downlink_handler()
- [ ] iot.c implements iot_report_immediate(), iot_report_bind(), iot_report_unbind()
- [ ] iot.c implements iot_trigger_ct_report(), iot_trigger_inverter_report()
- [ ] ml307r.h adds mqtt_set_downlink_callback() and mqtt_is_connected()
- [ ] ml307r.c implements callback mechanism and connects to iot_task
- [ ] Old wifi.c and wifi.h removed
- [ ] main.c updated to use iot instead of wifi
- [ ] Code compiles without errors

---

**Plan complete and saved to `docs/superpowers/plans/2026-04-14-iot-ml307r-implementation.md`**

Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
